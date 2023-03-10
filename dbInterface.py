import sqlite3

connection = sqlite3.connect("binTracking.db")
cursor = connection.cursor()

# create if not exists 

def setUp():
    # id is the arUCo tag number, bin is the bin number, x, y are cords and toTakeOut is hwehter bin has been
    # accomplished 
    cursor.execute("DROP Table Bins")
    cursor.execute("CREATE TABLE if not exists Bins(ARid INTEGER primary key, bin INTEGER, x REAL, y REAL, toTakeOut BOOL)")

# get bin, and return the (aruco id, binId, x, y cords of it and whether it should be taken out 
# should be 4 items 
def queryBin(binId):
    return cursor.execute(f"""
        SELECT * from Bins where bin = {binId}
    """).fetchall()

def insertRow(arUCoId, binId, x, y, toTakeOut):
    cursor.execute(
        f""" 
        Insert into Bins (ARid, bin, x, y, toTakeOut)
        Values({arUCoId}, {binId}, {x}, {y}, {toTakeOut})
        """
    )


if __name__ == "__main__":
    setUp() 
    insertRow(0, 0, 0, 0, False)
    print(queryBin(0))

