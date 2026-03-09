import sqlite3

DB_NAME = "warehouse.db"


# Open a connection to the db
def get_connection():
    connection = sqlite3.connect(DB_NAME)
    connection.row_factory = sqlite3.Row
    return connection