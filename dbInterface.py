import sqlite3
from enum import Enum



# COLUMN MACROS 
ARID_COLN = 0 
BIN_COLN = 1 
X_COLN = 2 
Y_COLN = 3 
TAKEOUT_COLN = 4
MISSING_COLN = 5




connection = sqlite3.connect("binTracking.db")
cursor = connection.cursor()

binTable = "Bins"
# create if not exists 

def setUp():
    # id is the arUCo tag number, bin is the bin number, x, y are cords and toTakeOut is hwehter bin has been
    # accomplished 
    cursor.execute(f"DROP Table {binTable}")
    cursor.execute("CREATE TABLE if not exists Bins(binId INTEGER primary key, curr_x REAL, curr_y REAL, TakeOut BOOL, Missing BOOL, take_out_x REAL, take_out_y REAL, home_x REAL, home_y REAL)")


def getFreeBins():
    return cursor.execute(f"""
                SELECT * from Bins where TakeOut = true AND Missing = false 
                   """).fetchall()

def getTakeHomeBins():
    return cursor.execute(f"""
        SELECT * from Bins where TakeOut = false AND Missing = false 
    """).fetchall()

# get bin, and return the (aruco id, binId, x, y cords of it and whether it should be taken out 
# should be 4 items 
def queryBin(binId):
    return cursor.execute(f"""
        SELECT * from Bins where binId = {binId}
    """).fetchall()

# x, y is curr loc, take_out is where to bring back, and home is original pos on map 
def insertRow(binId, x, y, toTakeOut = False, missing = False, take_out_x = 0, take_out_y = 0, home_x = None, home_y = None):
    cursor.execute(
        f""" 
        Insert into Bins (binId, curr_x, curr_y, TakeOut, Missing, take_out_x, take_out_y, home_x, home_y)
        Values({binId}, {x}, {y}, {toTakeOut}, {missing}, {take_out_x}, {take_out_y}, {home_x if home_x is not None else x}, {home_y if home_y is not None else y})
        """
    )

def updateTakeOut(binId, takeOut):
    cursor.execute( 
        f"""
        UPDATE Bins
            SET TakeOut = {takeOut}
        WHERE
            binId = {binId}
        """
    )

def updateLoc(binId, x, y):
    cursor.execute( 
        f"""
        UPDATE Bins
            SET curr_x = {x},
            curr_y = {y}
        WHERE
            binId = {binId}
        """
    ) 

def updateHomeLoc(binId, home_x, home_y):
    cursor.execute( 
        f"""
        UPDATE Bins
            SET home_x = {home_x},
            home_y = {home_y}
        WHERE
            binId = {binId}
        """
    ) 
    
def updateTakeOutLoc(binId, take_out_x, take_out_y):
    cursor.execute( 
        f"""
        UPDATE Bins
            SET take_out_x = {take_out_x},
            take_out_y = {take_out_y}
        WHERE
            binId = {binId}
        """
    ) 

def saveDbChanges():
    connection.commit() 


if __name__ == "__main__":
    setUp() 
    insertRow(0, 0.0, 0.0, False, False, 0, 0, 2.5, 0)
    # insertRow(1, 1, 1, False, False, 0, -1)
    # fetchall returns tuple of columns 
    print(queryBin(0))
    saveDbChanges() 
    updateLoc(0, 1, 1) 
    

    connection.close() 

