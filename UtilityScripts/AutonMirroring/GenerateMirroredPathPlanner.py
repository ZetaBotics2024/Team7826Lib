import json

file_path = "src\main\deploy\pathplanner\paths\ExampleAutonPath.path"
f = open(file_path, "r")
test = json.load(f)
print(test["waypoints"])
#print(f.read())
def main():
    pass

