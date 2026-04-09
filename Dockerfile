FROM cyberbotics/webots.cloud:R2025a-ubuntu22.04
ARG PROJECT_PATH
RUN mkdir -p $PROJECT_PATH
COPY . $PROJECT_PATH

# Install simulator dependencies
RUN if [ -f "$PROJECT_PATH/simulator/requirements.txt" ]; then pip3 install -r $PROJECT_PATH/simulator/requirements.txt; fi
# Install backend dependencies if needed for the simulation
RUN if [ -f "$PROJECT_PATH/backend/requirements.txt" ]; then pip3 install -r $PROJECT_PATH/backend/requirements.txt; fi
