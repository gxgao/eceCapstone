import dbInterface 
from enum import Enum
from typing import Optional

class BIN_SIDE(Enum):
    WIDE = 0 
    SLIM = 1 
    DIAGONAL = 2

class Bin:
    def __init__(self, binId, x, y, angle, takeOut, missing, take_out_x, take_out_y, take_out_angle, home_x, home_y, home_angle): 
        self.binId = binId 
        self.x = x 
        self.y = y 
        self.angle = angle
        self.takeOut = takeOut 
        self.missing = missing
        self.take_out_x = take_out_x
        self.take_out_y = take_out_y
        self.take_out_angle = take_out_angle
        self.home_x = home_x
        self.home_y = home_y 
        self.home_angle = home_angle

    def getProperties(self):
        return self.binId, self.x, self.y, self.angle, self.takeOut, self.missing, self.take_out_x, self.take_out_y, self.take_out_angle, self.home_x, self.home_y 
    
    def getXY(self):
        return self.x, self.y 

    def getCurPos(self):
        return self.x, self.y, self.angle

    def getTakeOutXY(self):
        return self.take_out_x, self.take_out_y 

    def getTakeOutPos(self):
        return self.take_out_x, self.take_out_y, self.take_out_angle

    def getHomeXY(self):
        return self.home_x, self.home_y 

    def getHomePos(self):
        return self.home_x, self.home_y, self.home_angle

    def __repr__(self):
        return f"Bin: {self.binId}, x: {self.x}, y: {self.y}, {self.angle=}, takeOut? : {self.takeOut}, take_out_x : {self.take_out_x}, take_out_y : {self.take_out_y},  home_x {self.home_x}, home_y : {self.home_y} "

class BinTracker: 

    def __init__(self):
        self.currBin: Optional[Bin] = None 
        self.binToGet = None 
        self.binDir: dict[int, Bin] = {} 

    def setupDb(self):
        dbInterface.setUp() 
        dbInterface.insertRow(0, 2.0,0, -.038, True, False, 0, 0, 0)
        dbInterface.insertRow(1, 1, 1,0, False, False, 0, -1,0)
        # fetchall returns tuple of columns 
        dbInterface.saveDbChanges() 

    
    def getTakeOutBin(self):
        takeHomeBins = dbInterface.getTakeHomeBins()
        if (len(takeHomeBins) == 0):
            return None 
        bin_list_representation = takeHomeBins[0] 
        binId = bin_list_representation[0]
        if binId in self.binDir:
            return self.binDir[binId] 

        self.binToGet = Bin(*bin_list_representation)
        return self.binToGet 


    def getFreeBin(self):
        emptyBins = dbInterface.getFreeBins()
        if (len(emptyBins) == 0):
            return None 
        bin_list_repr = emptyBins[0] 
        self.binToGet = Bin(*bin_list_repr)
        return self.binToGet 
    
    def SetBin(self, id):
        bin_list_reprs = dbInterface.queryBin(id)
        if bin_list_reprs is None or len(bin_list_reprs) == 0:
            return None
        bin_list_repr = bin_list_reprs[0]
        self.binToGet = Bin(*bin_list_repr)
        return self.binToGet


    def ArucoIdToBinId(self, aruco_id): 
        return aruco_id // 3
    
    def GetBinSide(self, aruco_id):
        return BIN_SIDE(aruco_id % 3) 
    
    def SetMissing(self, binId, missing):
        dbInterface.updateMissing(binId, missing)
        if binId in self.binDir:
            self.binDir[binId].missing = missing

    # takes list of ids and queries database and returns list of bins 
    def checkIds(self, markerIds):
        dbBins = [] 
        for mId in markerIds: 
            dbBins += dbInterface.queryBin(mId[0])
        bins = [] 
        for bin_list_repr in dbBins: 
            bins.append(Bin(*bin_list_repr))
        return bins  
    
    def binPickUp(self, binToPick):

        self.currBin = binToPick
        self.binDir[self.currBin.binId] = binToPick
    
    def binDrop(self, needToTakeOut, x, y, angle):
        binId = self.currBin.getProperties()[0] 
    
        self.currBin.takeOut = needToTakeOut 
        dbInterface.updateTakeOut(binId, self.currBin.takeOut)
        # either way we nee to update the location since we dropped the bin 
        dbInterface.updateLoc(binId, x, y, angle) 
        dbInterface.saveDbChanges() 
    

        droppedBin = self.currBin
        self.currBin.x = x 
        self.currBin.y = y 
        self.currBin.angle = angle
        self.currBin = None 
        return droppedBin 
    

if __name__ == "__main__":
    bt = BinTracker() 
    freeBin = bt.getFreeBin() 
    print(freeBin) 




