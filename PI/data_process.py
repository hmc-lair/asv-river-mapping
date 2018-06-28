
# get data
f = open('data_ensemble.txt', 'r')
data = f.readline()[:-1] # this data exlucded the first 4 bytes in an ensemble

# experiment
	# the 5th byte is unsepcified 
num_data_type = data[2]
print(data.encode())