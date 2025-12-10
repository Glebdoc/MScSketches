# Welcome to my LLM

# How to run a drone case: 
	1. Go to trimmer_drone.py 
	2. Specify the path to the folder where the config file is (line marked as TODO)
	stored (this will be the main working folder for the case)
	3. Copy "base_drone.json" file from ./config into the created directory
	4. Adjust the settings in the "base_drone.json" 	
		IMPORTANT: change "output_dir": value to PATH to a newly created folder. 
		Rename the copied "base_drone.json" to "_.json"

	5. Uncomment the desired trimmer 
	6. Run the file from MSCSKETCHES directory 
	7. After the run is complete there will be 
	"_res.npz" and "performance.json" files present in the created directory

# How to run a helicopter case:
	1. Do trimmer_helicopter.py 
	2. --//--
	3. Copy "base_helicopter.json" file from ./config into the created directory
	4. --//--
	5. --//--
	6. --//--
	7. --//--

		