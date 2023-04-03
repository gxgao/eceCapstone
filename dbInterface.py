import sqlite3
from enum import Enum



# COLUMN MACROS 
ARID_COLN = 0 
BIN_COLN = 1 
X_COLN = 2 
Y_COLN = 3 
TAKEOUT_COLN = 4




connection = sqlite3.connect("binTracking.db")
cursor = connection.cursor()

binTable = "Bins"
# create if not exists 

def setUp():
    # id is the arUCo tag number, bin is the bin number, x, y are cords and toTakeOut is hwehter bin has been
    # accomplished 
    cursor.execute(f"DROP Table {binTable}")
    cursor.execute("CREATE TABLE if not exists Bins(ARid INTEGER primary key, bin INTEGER, x REAL, y REAL, TakeOut BOOL)")

# get bin, and return the (aruco id, binId, x, y cords of it and whether it should be taken out 
# should be 4 items 
def queryBin(binId):
    return cursor.execute(f"""
        SELECT * from Bins where bin = {binId}
    """).fetchall()

def queryAr(arId):
    return cursor.execute(f"""
        SELECT * from Bins where ARid = {arId}
    """).fetchall()

def insertRow(arucoId, binId, x, y, toTakeOut):
    cursor.execute(
        f""" 
        Insert into Bins (ARid, bin, x, y, TakeOut)
        Values({arucoId}, {binId}, {x}, {y}, {toTakeOut})
        """
    )

def updateTakeOut(arucoId, takeOut):
    cursor.execute( 
        f"""
        UPDATE table
            SET TakeOut = {takeOut}
        WHERE
            ARid = {arucoId}
        """
    )
    
def updateLoc(arucoId, x, y):
    cursor.execute( 
        f"""
        UPDATE table
            SET x = {x}
            SET y = {y}
        WHERE
            ARid = {arucoId}
        """
    ) 
    
def saveDbChanges():
    connection.commit() 




if __name__ == "__main__":
    setUp() 
    insertRow(0, 0, 0, 0, False)

    print(queryBin(0))
    saveDbChanges() 
    connection.close() 

