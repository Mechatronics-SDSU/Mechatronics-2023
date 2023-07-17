import os

folder_path = "path/to/folder"  # Replace with the actual folder path
new_letter = "A"  # Replace with the desired letter to be added

# Get all files in the folder
files = os.listdir(folder_path)

for file_name in files:
    if file_name.endswith((".jpg", ".jpeg", ".png")):  # Specify the image file extensions you want to rename
        # Split the file name and extension
        name, extension = os.path.splitext(file_name)

        # Add the new letter to the name
        new_name = name + new_letter + extension

        # Rename the file
        os.rename(os.path.join(folder_path, file_name), os.path.join(folder_path, new_name))
        print(f"Renamed {file_name} to {new_name}")