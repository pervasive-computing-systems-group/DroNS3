import random
import string

def generate_data(num_bytes, destination_file):
    with open(destination_file, "w") as output_file:
        output_file.write("Beginning_")
        for i in range(num_bytes - len("Beginning_") - len("_END")):
            char = random.choice(string.ascii_letters)
            output_file.write(char)
        output_file.write("_END")
    print("Number of bytes sent: " + str(num_bytes))