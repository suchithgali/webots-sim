// autopilot_node.cpp
//
// Mavic 2 Pro waypoint mission for Webots.
// Takes off, flies through each hardcoded waypoint in order,
// returns to the takeoff position, and lands.
//
// Topics (all under /Mavic_2_PRO/ namespace):
//   sub  gps_position   geometry_msgs/PointStamped   GPS from driver
//   sub  heading        std_msgs/Float32             IMU yaw from driver
//   pub  cmd_vel        geometry_msgs/Twist          velocity + altitude cmd

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cmath>
#include <vector>
#include <array>

// ── Tuning ─────────────────────────────────────────────────────────────────
static constexpr double CRUISE_ALT = 4.0;   // cruise altitude (m)
static constexpr double KP_XY     = 0.6;    // position error → speed (m/s per m)
static constexpr double MAX_SPEED = 2.0;    // horizontal speed cap (m/s)
static constexpr double ARRIVE_R  = 1.5;    // waypoint capture radius (m)
static constexpr double LAND_R    = 0.8;    // landing position capture radius (m)
// ───────────────────────────────────────────────────────────────────────────

enum class State { WAIT_GPS, TAKEOFF, NAVIGATE, RETURN_HOME, LAND, DONE };

class AutopilotNode : public rclcpp::Node
{
public:
  AutopilotNode() : Node("autopilot_node"), state_(State::WAIT_GPS),
                    wp_(0), yaw_(0), got_pos_(false)
  {
    using std::placeholders::_1;
    const std::string ns = "/Mavic_2_PRO/";

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(ns + "cmd_vel", 10);

    gps_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      ns + "gps_position", 10, std::bind(&AutopilotNode::gps_cb, this, _1));

    hdg_sub_ = create_subscription<std_msgs::msg::Float32>(
      ns + "heading", 10, std::bind(&AutopilotNode::hdg_cb, this, _1));

    // 20 Hz control loop
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AutopilotNode::loop, this));

    // ── Hardcoded waypoints [x, y] at cruise altitude ──────────────────
    // Modify these to define the path the drone should follow.
    wps_ = {
      { 7.41166, 2.71991},   // 0  aisle A-B east
      { 3.50,  1.00 },   // 1  aisle A-B west
      { -0.07,  0.86 },   // 2  aisle B-C west
      { -3.89,  0.85 },   // 3  aisle B-C east
      { -7.15,  0.85 },   // 4  aisle C-D east
      {7.41166, 2.71991},   // 5  aisle C-D west
    };

    RCLCPP_INFO(get_logger(), "Autopilot ready — waiting for GPS fix...");
  }

private:
  // ── Callbacks ──────────────────────────────────────────────────────────
  void gps_cb(const geometry_msgs::msg::PointStamped::SharedPtr m)
  {
    px_ = m->point.x;
    py_ = m->point.y;
    pz_ = m->point.z;

    if (!got_pos_) {
      got_pos_ = true;
      // Record takeoff position so we can return here later
      home_x_ = px_;
      home_y_ = py_;
      state_  = State::TAKEOFF;
      RCLCPP_INFO(get_logger(),
        "GPS fix acquired at (%.2f, %.2f, %.2f) — taking off", px_, py_, pz_);
    }
  }

  void hdg_cb(const std_msgs::msg::Float32::SharedPtr m)
  {
    yaw_ = static_cast<double>(m->data);
  }

  // ── Helpers ────────────────────────────────────────────────────────────
  void fly_toward(double tx, double ty, double tz,
                  geometry_msgs::msg::Twist& cmd)
  {
    double vx = KP_XY * (tx - px_);
    double vy = KP_XY * (ty - py_);
    double spd = std::hypot(vx, vy);
    if (spd > MAX_SPEED) { vx *= MAX_SPEED / spd; vy *= MAX_SPEED / spd; }

    cmd.linear.x  = vx;
    cmd.linear.y  = vy;
    cmd.linear.z  = tz;       // absolute altitude target
    cmd.angular.z = 0.0;
  }

  // ── Main control loop ─────────────────────────────────────────────────
  void loop()
  {
    geometry_msgs::msg::Twist cmd;

    switch (state_) {

    // ▸ Wait for first GPS reading ─────────────────────────────────────
    case State::WAIT_GPS:
      cmd_pub_->publish(cmd);   // zero velocity
      return;

    // ▸ Climb straight up to cruise altitude ───────────────────────────
    case State::TAKEOFF:
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Taking off... alt=%.2f / %.2f m", pz_, CRUISE_ALT);

      cmd.linear.z = CRUISE_ALT;
      cmd_pub_->publish(cmd);

      if (pz_ >= CRUISE_ALT - 0.5) {
        state_ = State::NAVIGATE;
        wp_ = 0;
        RCLCPP_INFO(get_logger(),
          "Cruise altitude reached — heading to WP 0");
      }
      return;
    }

    // ▸ Fly through each waypoint in order ─────────────────────────────
    case State::NAVIGATE:
    {
      const auto& wp = wps_[wp_];
      double dist = std::hypot(wp[0] - px_, wp[1] - py_);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[WP %zu/%zu] pos=(%.2f,%.2f,%.2f) target=(%.2f,%.2f) dist=%.2fm",
        wp_, wps_.size() - 1, px_, py_, pz_, wp[0], wp[1], dist);

      if (dist < ARRIVE_R) {
        RCLCPP_INFO(get_logger(), ">>> WP %zu reached", wp_);
        wp_++;
        if (wp_ >= wps_.size()) {
          state_ = State::RETURN_HOME;
          RCLCPP_INFO(get_logger(),
            "All waypoints visited — returning home (%.2f, %.2f)",
            home_x_, home_y_);
        }
        return;
      }

      fly_toward(wp[0], wp[1], CRUISE_ALT, cmd);
      cmd_pub_->publish(cmd);
      return;
    }

    // ▸ Fly back to takeoff position at cruise altitude ────────────────
    case State::RETURN_HOME:
    {
      double dist = std::hypot(home_x_ - px_, home_y_ - py_);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "Returning home... dist=%.2fm", dist);

      if (dist < LAND_R) {
        state_ = State::LAND;
        RCLCPP_INFO(get_logger(), "Over home position — landing");
        return;
      }

      fly_toward(home_x_, home_y_, CRUISE_ALT, cmd);
      cmd_pub_->publish(cmd);
      return;
    }

    // ▸ Descend to the ground ──────────────────────────────────────────
    case State::LAND:
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "Landing... alt=%.2f m", pz_);

      // Command zero altitude to descend; keep correcting XY
      fly_toward(home_x_, home_y_, 0.0, cmd);
      cmd_pub_->publish(cmd);

      if (pz_ < 0.3) {
        state_ = State::DONE;
        RCLCPP_INFO(get_logger(), "Landed! Mission complete.");
      }
      return;
    }

    // ▸ Motors off, do nothing ─────────────────────────────────────────
    case State::DONE:
      cmd_pub_->publish(cmd);   // zero velocity
      return;
    }
  }

  // ── Members ────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr            cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr  gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr            hdg_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_;
  std::vector<std::array<double,2>> wps_;   // [x, y] waypoints
  std::size_t wp_;
  double px_{0}, py_{0}, pz_{0}, yaw_;
  double home_x_{0}, home_y_{0};            // takeoff position
  bool got_pos_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutopilotNode>());
  rclcpp::shutdown();
}