import abc
import defines
import math
import subprocess as sb
'''
    Solver parent class:
        Solvers will implement different planning/pathing algorithms for use during Misisons.

        __init__ : initialization method. All preprocessing should occur here.
                args: a list of all required parameters. Solver should gracefully exit if the correct arguments are not passed.

        solve: solution method, should return the expected solution.
'''
class Solver(object):
    __metaclass__ = abc.ABCMeta
    
    
    @abc.abstractmethod
    def __init__(self, args):
        pass

    @abc.abstractmethod
    def solve(self):
        pass


'''
    LKH_Solver is a child class of the solver class that utilizes the LKH TSP (Traveling Salesman Problem) solver to solve Fixed Hamiltonian Path Problems. 
    __init__ :
        Arguments:  
            args: an array of arguments. The arguments array should contain exactly:
                hpp_points: a list of ordered pair coordinates
                start: the index of the starting point in hpp_points
                end: the index of the ending point in hpp_points
                holder: The Process_Thread_Holder object for the mission
    solve: returns a list of ordered pairs in the order that the LKH solver proposes.
'''
class LKH_Solver(Solver):

    def __init__(self, args):

        self.arguments = args

        if len(args) != 4:
            raise Exception("Incorrect number of arguments, arguments should be: [hpp_points, start, end, holder].")
        
        self.points = self.arguments[0]
        self.start = self.arguments[1]
        self.end = self.arguments[2]
        self.holder = self.arguments[3]

        self.PrintLKHFile()

    def solve(self):
        return self.getLKHSolution()


    def PrintLKHFile(self):
            with open("FixedHPP.par", "w") as parFile:
                parFile.write("PROBLEM_FILE = FixedHPP.tsp\n")
                parFile.write("COMMENT Fixed Hamiltonian Path Problem\n")
                parFile.write("TOUR_FILE = LKH_output.dat\n")

            with open("FixedHPP.tsp", "w") as dataFile:
                dataFile.write("NAME : FixedHPP \n")
                dataFile.write("COMMENT : Fixed Hamiltonian Path Problem \n")
                dataFile.write("TYPE : TSP \n")
                dataFile.write("DIMENSION : " + str(len(self.points)) + "\n")
                dataFile.write("EDGE_WEIGHT_TYPE : EXPLICIT \n")
                dataFile.write("EDGE_WEIGHT_FORMAT : FULL_MATRIX\n")

                dataFile.write("EDGE_WEIGHT_SECTION\n")
                for i in range(len(self.points)):
                    for j in range(len(self.points)):
                        if(i == self.start and j == self.end) or (i == self.end and j == self.start):
                            dataFile.write("0.0\t")
                        else:
                            if i == self.start or i == self.end or j == self.start or j == self.end:
                                dataFile.write(str(self.FindNodeDistance(self.points[i], self.points[j])*1000) + "\t")
                            else:
                                dataFile.write(str(self.FindNodeDistance(self.points[i], self.points[j])) + "\t")
                    dataFile.write("\n")
                dataFile.write("EOF\n")
	 #Finds the distance between two points
    def FindNodeDistance(self, a, b):
        return math.sqrt(((a[0] - b[0])**2 + (a[1] - b[1])**2))

	#Runs the LKH solver and returns the solution
    def getLKHSolution(self):
        lkh_process = sb.Popen([defines.LKH_PATH, "FixedHPP.par"])
        self.holder.add_process(lkh_process)


        # process_num = ph.add_process(lkh_process)
        lkh_process.communicate()[0]
        # ph.remove_process(process_num)

        with open("LKH_output.dat") as outputFile:
            lines = outputFile.readlines()
        lkh_path = []
        
        for i in range(6, len(lines) - 1):
            if int(lines[i]) != -1:
                lkh_path.append(int(lines[i]) -1)


        print(lkh_path)
        if lkh_path[0] != self.start or lkh_path[-1] != self.end:
            
            if lkh_path[1] == self.end and lkh_path[0] == self.start:
                lkh_path.append(lkh_path[0])
                lkh_path.pop(0)
                lkh_path.reverse()
            
            while lkh_path[0] != self.start:
                print("Rotating list...")
                lkh_path.append(lkh_path[0])
                lkh_path.pop(0)

            if lkh_path[1] == self.end:
                lkh_path.append(lkh_path[0])
                lkh_path.pop(0)
                lkh_path.reverse()

            if lkh_path[-1] != self.end:
                raise(Exception("Temp exception 2 for TSP failing to function as HPP"))
        

        return lkh_path
