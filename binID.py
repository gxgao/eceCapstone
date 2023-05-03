import dbInterface 
from enum import Enum

class BIN_SIDE(Enum):
    SLIM = 0 
    WIDE = 1 



class Bin:
    def __init__(self, binId, x, y, takeOut, missing): 
        self.binId = binId 
        self.x = x 
        self.y = y 
        self.takeOut = takeOut 
        self.missing = missing

    def getProperties(self):
        return self.arucoId, self.binId, self.x, self.y, self.takeOut, self.dir  
    
    def getXY(self):
        return self.x, self.y 

    def __repr__(self):
        return f"Bin: {self.binId}, x: {self.x}, y: {self.y}, takeOut? : {self.takeOut}"

class BinTracker: 

    def __init__(self):
        self.currBin = None 
        self.binToGet = None 

    def getFreeBin(self):
        emptyBins = dbInterface.getFreeBins()
        if (len(emptyBins) == 0):
            return None 
        _, binId, x, y, takeOut, missing = emptyBins[0] 
        self.binToGet = Bin(binId, x, y, takeOut, missing)
        return self.binToGet 

    def ArucoIdToBinId(self, aruco_id): 
        return aruco_id // 2 
    
    def GetBinSide(self, aruco_id):
        return BIN_SIDE(aruco_id % 2) 
    

    # takes list of ids and queries database and returns list of bins 
    def checkIds(self, markerIds):
        dbBins = [] 
        for mId in markerIds: 
            dbBins += dbInterface.queryBin(mId[0])
        bins = [] 
        for aruco, binId, x, y, takeOut, _ in dbBins: 

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
    

    

