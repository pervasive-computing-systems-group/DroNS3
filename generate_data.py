import random
import string

def generate_data(num_bytes, destination_file):
    with open(destination_file, "w") as output_file:
        for i in range(num_bytes):
            char = random.choice(string.ascii_letters)
            output_file.write(char)