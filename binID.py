import dbInterface 
from enum import Enum

class BIN_DIR(Enum):
    FRONT = 0
    BACK = 1
    RIGHT = 2
    LEFT = 3 


ENUMDIR = Color = [BIN_DIR.FRONT, BIN_DIR.BACK, BIN_DIR.RIGHT, BIN_DIR.LEFT]

class Bin:
    def __init__(self, arucoId, binId, x, y, takeOut): 
        self.arucoId = arucoId 
        self.binId = binId 
        self.x = x 
        self.y = y 
        self.takeOut = takeOut 
        self.dir = ENUMDIR[arucoId % 4] 

    def getProperties(self):
        return self.arucoId, self.binId, self.x, self.y, self.takeOut, self.dir  
    
    def getDir(self):
        return self.dir 

class BinTracker: 

    def __init__(self):
        self.currBin = None 

    # takes list of ids and queries database and returns list of bins 
    def checkIds(self, markerIds):
        dbBins = [] 
        for mId in markerIds: 
            print(mId[0])
            dbBins += dbInterface.queryBin(mId[0])
        bins = [] 
        for aruco, binId, x, y, takeOut in dbBins: 

            bins.append(Bin(aruco, binId, x, y, takeOut))

        return bins  
    
    def binPickUp(self, binToPick):
        self.currBin = binToPick 
    
    def binDrop(self, finished, x, y):
        arucoId = self.currBin.getProperties()[0] 
        if finished: 
            dbInterface.updateTakeOut(arucoId, False)
        # either way we nee to update the location since we dropped the bin 
        dbInterface.updateLoc(arucoId, x, y) 
        dbInterface.saveDbChanges() 

        droppedBin = self.currBin
        self.currBin = None 
        return droppedBin 
    

    

