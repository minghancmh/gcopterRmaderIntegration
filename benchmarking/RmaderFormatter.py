

class RmaderFormatter:

    def __init__(self, filePath):
        self.filePath = filePath
        self.fullArr = []
        # print("formatter initialized")

    def format(self):

        import re
        import numpy as np

        with open(self.filePath) as f:
            lines = f.readlines()
            text = ''.join(lines)
            split_text = re.split(r"=+Running Octopus Search=+", text)
            timeList = []   
            for text in split_text:
                waypoint_pattern = r'-------------Waypoint------------\n\s*([\d.-]+)\n\s*([\d.-]+)\n\s*([\d.-]+)'
                waypoint_matches = re.findall(waypoint_pattern, text)
                # print(waypoint_matches)
                for waypoint_match in waypoint_matches:
                    if float(waypoint_match[0]) > 9.5 and float(waypoint_match[1]) < -9.5:
                        patternForTime = r"Time for planner algorithm.*?(\d+\.?\d*)ms"
                        time = re.findall(patternForTime, text)
                        timeList.append(float(time[1]))

        for i in range(52-len(timeList)):
            timeList.append(np.mean(timeList))
        self.fullArr = timeList
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

        