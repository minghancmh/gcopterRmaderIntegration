class GcopterFormatter:

    def __init__(self, filePath):
        self.filePath = filePath
        self.fullArr = []
        # print("formatter initialized")

    def format(self):
        import re

        with open(self.filePath) as f:
            lines = f.readlines()


        for line in lines:
            if line[0] == "T": 
                s = ""
                # for char in line:
                text_temp = re.findall("(\d+)*(\.\d+)", line)
                s += text_temp[0][0]
                s += text_temp[0][1]
                self.fullArr.append(float(s))

        # print("Number of Samples: ", len(out))

        return self.fullArr
    
    def changeFilePath(self, filePath):
        self.filePath = filePath
        self.fullArr = []

    def getFilePath(self):
        return self.filePath

    def removeOutliers(self):
        import numpy as np
        u = np.mean(self.fullArr)
        s = np.std(self.fullArr)
        noOutliers = [e for e in self.fullArr if (u-1.5*s < e < u+1.5*s)]

        for i in range(len(self.fullArr) - len(noOutliers)):
            noOutliers.append(u)
        # print("nooutliers: ", len(noOutliers))
        # print("arr: ", len(self.fullArr))

        return noOutliers





                
