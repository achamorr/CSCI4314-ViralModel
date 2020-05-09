
import os

if __name__ == "__main__":
    
    datadir = "data"
    #get all files in dir
    allfiles = list()
    for (dirpath, dirnames, filenames) in os.walk(datadir):
        #print (dirpath)
        filenames = [os.path.join(dirpath, f) for f in filenames]
        allfiles.extend(filenames)
    #allfiles= ["data\\pre-runs\\flockingagents_N=5.7_r0=2_in=10_frames=2460"]
    #print(allfiles)
    #print (len(allfiles))

    
    for file in allfiles:
        filename = os.path.basename(file)
        dirname = os.path.dirname(file)
        with open(file, "r+") as fp:
            rename = os.path.join(dirname, ("2_" + filename))
            with open(rename, "w+") as wfp:
                lineread = fp.readline()
                linelist = lineread.split(',')
                line = ",".join(linelist[:2]) #merge first two columns
                if (line is not None):
                    if ("\n" not in line):
                        line += "\n"
                    wfp.write(line)
                while lineread:
                    lineread = fp.readline()
                    linelist = lineread.split(',')
                    line = ",".join(linelist[:2]) #merge first two columns
                    print (line)
                    if (line is not None):
                        if ("\n" not in line):
                            line += "\n"
                        wfp.write(line)
