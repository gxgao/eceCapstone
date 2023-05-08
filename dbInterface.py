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
    cursor.execute("CREATE TABLE if not exists Bins(binId INTEGER primary key, curr_x REAL, curr_y REAL, curr_angle REAL, TakeOut BOOL, Missing BOOL, take_out_x REAL, take_out_y REAL, take_out_angle REAL, home_x REAL, home_y REAL, home_angle REAL)")


def getFreeBins():
    return cursor.execute(f"""
                SELECT * from Bins where TakeOut = true AND Missing = false ORDER BY binId
                   """).fetchall()

def getTakeHomeBins():
    return cursor.execute(f"""
        SELECT * from Bins where TakeOut = false AND Missing = false ORDER BY binId DESC 
    """).fetchall()

# get bin, and return the (aruco id, binId, x, y cords of it and whether it should be taken out 
# should be 4 items 
def queryBin(binId):
    return cursor.execute(f"""
        SELECT * from Bins where binId = {binId}
    """).fetchall()

# x, y is curr loc, take_out is where to bring back, and home is original pos on map 
def insertRow(binId, x, y, angle, toTakeOut = False, missing = False, take_out_x = 0, take_out_y = 0, take_out_angle = 0, home_x = None, home_y = None, home_angle = None):
    cursor.execute(
        f""" 
        Insert into Bins (binId, curr_x, curr_y, curr_angle, TakeOut, Missing, take_out_x, take_out_y, take_out_angle, home_x, home_y, home_angle)
        Values({binId}, {x}, {y}, {angle}, {toTakeOut}, {missing}, {take_out_x}, {take_out_y}, {take_out_angle},  {home_x if home_x is not None else x}, {home_y if home_y is not None else y}, {home_angle if home_angle is not None else angle})
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
    
def updateMissing(binId, missing):
    cursor.execute( 
        f"""
        UPDATE Bins
            SET Missing = {missing}
        WHERE
            binId = {binId}
        """
    )

def updateLoc(binId, x, y, angle):
    cursor.execute( 
        f"""
        UPDATE Bins
            SET curr_x = {x},
            curr_y = {y},
            curr_angle = {angle}
        WHERE
            binId = {binId}
        """
    ) 

def updateHomeLoc(binId, home_x, home_y, home_angle):
    cursor.execute( 
        f"""
        UPDATE Bins
            SET home_x = {home_x},
            home_y = {home_y},
            home_angle = {home_angle}
        WHERE
            binId = {binId}
        """
    ) 
    
def updateTakeOutLoc(binId, take_out_x, take_out_y, take_out_angle):
    cursor.execute( 
        f"""
        UPDATE Bins
            SET take_out_x = {take_out_x},
            take_out_y = {take_out_y},
            take_out_angle = {take_out_angle}
        WHERE
            binId = {binId}
        """
    ) 

def saveDbChanges():
    connection.commit() 


if __name__ == "__main__":
    setUp() 
    insertRow(0, 2.4, 0, -3.14, True, False, 0, 0, 0)
    insertRow(1, 2.4, -2.0, 3.14/2, True, False, 0, -.7, 0)
    
    # insertRow(1, 1, 1, False, False, 0, -1)
    # fetchall returns tuple of columns 
    print(getFreeBins())
    # print(queryBin(1))
    saveDbChanges() 
    # updateLoc(0, 1, 1) 
    

    connection.close() 

