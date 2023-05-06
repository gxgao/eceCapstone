import dbInterface 
from enum import Enum
from typing import Optional

class BIN_SIDE(Enum):
    SLIM = 0 
    WIDE = 1 



class Bin:
    def __init__(self, binId, x, y, takeOut, missing, take_out_x, take_out_y, home_x, home_y): 
        self.binId = binId 
        self.x = x 
        self.y = y 
        self.takeOut = takeOut 
        self.missing = missing
        self.take_out_x = take_out_x
        self.take_out_y = take_out_y
        self.home_x = home_x
        self.home_y = home_y 

    def getProperties(self):
        return self.binId, self.x, self.y, self.takeOut, self.takeOut, self.missing, self.take_out_x, self.take_out_y  
    
    def getXY(self):
        return self.x, self.y 

    def getTakeOutXY(self):
        return self.take_out_x, self.take_out_y 
        
    def getHomeXY(self):
        return self.home_x, self.home_y 

    def __repr__(self):
        return f"Bin: {self.binId}, x: {self.x}, y: {self.y}, takeOut? : {self.takeOut}, take_out_x : {self.take_out_x}, take_out_y : {self.take_out_y}, home_x {self.home_x}, home_y : {self.home_y} "

class BinTracker: 

    def __init__(self):
        self.currBin: Optional[Bin] = None 
        self.binToGet = None 
        self.binDir = {} 

    def setupDb(self):
        dbInterface.setUp() 
        dbInterface.insertRow(0, 2.0, -.038, True, False, 0, 0)
        dbInterface.insertRow(1, 1, 1, False, False, 0, -1)
        # fetchall returns tuple of columns 
        dbInterface.saveDbChanges() 

    
    def getTakeOutBin(self):
        takeHomeBins = dbInterface.getTakeHomeBins()
        print("takeHomeBins", takeHomeBins)
        if (len(takeHomeBins) == 0):
            return None 
        binId, x, y, takeOut, missing, take_out_x, take_out_y, home_x, home_y = takeHomeBins[0] 
        if binId in self.binDir:
            return self.binDir 

        self.binToGet = Bin(binId, x, y, takeOut, missing, take_out_x, take_out_y, home_x, home_y)
        return self.binToGet 


    def getFreeBin(self):
        emptyBins = dbInterface.getFreeBins()
        if (len(emptyBins) == 0):
            return None 
        binId, x, y, takeOut, missing, take_out_x, take_out_y, home_x, home_y = emptyBins[0] 
        self.binToGet = Bin(binId, x, y, takeOut, missing, take_out_x, take_out_y, home_x, home_y)
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
        for binId, x, y, takeOut, _, take_out_x, take_out_y, home_x, home_y in dbBins: 

            bins.append(Bin(aruco, binId, x, y, takeOut, take_out_x, take_out_y, home_x, home_y))

        return bins  
    
    def binPickUp(self, binToPick):

        self.currBin = binToPick
        self.binDir[self.currBin.binId] = binToPick
    
    def binDrop(self, needToTakeOut, x, y):
        binId = self.currBin.getProperties()[0] 
    
        self.currBin.takeOut = needToTakeOut 
        dbInterface.updateTakeOut(binId, self.currBin.takeOut)
        # either way we nee to update the location since we dropped the bin 
        dbInterface.updateLoc(binId, x, y) 
        dbInterface.saveDbChanges() 
    

        droppedBin = self.currBin
        self.currBin.x = x 
        self.currBin.y = y 
        self.currBin = None 
        return droppedBin 
    

if __name__ == "__main__":
    bt = BinTracker() 
    freeBin = bt.getFreeBin() 
    print(freeBin) 




