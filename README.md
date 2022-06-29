# Vision and Language Navigation | IIIT Hyderabad

Steps: 
1. Run `chmod +x clean.sh` and `./clean.sh` whenever you want to clean the dataset and start afresh.
2. Run `roscore`
3. python3 `begin_dataset_building.py` to start building the dataset.

To stop the python file, use the `ps -a` to get the PID of python file and shut it down using `kill -9 PID` where PID is the process identity.
