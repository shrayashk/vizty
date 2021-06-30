import os
from tkinter import filedialog
from tkinter import *
import pathlib
import open3d as o3d
import numpy as np
from tkinter import messagebox
import io
import json


pc_content = ''
file_path = ''
label_path = ''
label_content = []
bboxes=[]
corpus=[]
vertices = []
lines = [[0, 1], [1, 2], [2, 3], [0, 3],
         [4, 5], [5, 6], [6, 7], [4, 7],
         [0, 4], [1, 5], [2, 6], [3, 7]]

colors_0 = [[1, 0, 0] for _ in range(len(lines))]
colors_1 = [[0, 0, 1] for _ in range(len(lines))]


def convert_to_8corners(theta,x,y,z,l,w,h):

        rot = np.array([[ np.cos(theta),  -np.sin(theta),                  0],
                        [ np.sin(theta),   np.cos(theta),                  0],
                        [                  0,                    0,                  1]])
        x_corners = [ -l/2,  -l/2,  l/2,  l/2,
                      -l/2,  -l/2,  l/2,  l/2]
        y_corners = [  w/2,  -w/2, -w/2,  w/2,
                       w/2,  -w/2, -w/2,  w/2]
        z_corners = [  h/2,   h/2,  h/2,  h/2,
                      -h/2,  -h/2, -h/2, -h/2]
        
        corners_3d = np.dot(rot, np.vstack([x_corners, y_corners, z_corners]))
        
        corners_3d[0, :] = corners_3d[0, :] + x
        corners_3d[1, :] = corners_3d[1, :] + y
        corners_3d[2, :] = corners_3d[2, :] + z
        return np.transpose(corners_3d)

def open_pc_file():
    global pc_content
    global file_path
    filetypes = (
        ('.BIN', '*.bin'),
        ('.PCD', '*.pcd'),
        ('All files', '*.*')
    )
    filename = filedialog.askopenfilename(filetypes=filetypes)
    file_path = os.path.realpath(filename)
    f_ext=pathlib.Path(file_path).suffix
    if f_ext == '.bin':
        bin_pcd = np.fromfile(file_path, dtype=np.float32)
        points = bin_pcd.reshape((-1, 4))[:, 0:3]
        pc_content = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    elif f_ext == '.pcd':
        pc_content=o3d.io.read_point_cloud(file_path)
    else:
        messagebox.showerror("ERROR", "INVALID FILE EXTENSION")
    pc_entry.delete(0, END)
    pc_entry.insert(0, file_path)
    return pc_content

def label_handler(label_path):
    with io.open(label_path, mode="r") as f:
        for line in f:
            corpus.append(line.split(','))
    for i in range(len(corpus)):
        class_id = int(corpus[i][0])
        x = float(corpus[i][1])
        y = float(corpus[i][2])
        z = float(corpus[i][3])
        theta = float(corpus[i][4])
        w = float(corpus[i][5])
        l = float(corpus[i][6])
        h = float(corpus[i][7])
        vertices.append(convert_to_8corners(theta,x,y,z,l,w,h))
    for i in range(len(corpus)):
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(vertices[i])
        line_set.lines = o3d.utility.Vector2iVector(lines)
        if int(corpus[i][0]) == 1 :
            line_set.colors = o3d.utility.Vector3dVector(colors_1)
        elif int(corpus[i][0]) == 0 :
            line_set.colors = o3d.utility.Vector3dVector(colors_0)
        label_content.append(line_set)
    return label_content
    
def visualize_pc_file(pc_content,label_content):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pc_content)
    if len(label_content) > 0:
        for i in label_content:
            vis.add_geometry(i)
    vis.run()

def json_to_kitti(json_path):
    json_data = None
    with open(json_path) as annotation:
        json_data = json.load(annotation)
    box_list = json_data["frame"]["bounding_boxes"]
    for box in box_list:
        kitti_label=[box["kitti_id"], box["kitti_x"], box["kitti_y"], box["kitti_z"], box["kitti_theta"], box["kitti_w"], box["kitti_l"], box["kitti_h"]]
        bboxes.append([round(num, 4) for num in kitti_label])
        return bboxes

def json_handler(bboxes):
    for i in range(len(bboxes)):
        class_id = int(bboxes[i][0])
        x = float(bboxes[i][1])
        y = float(bboxes[i][2])
        z = float(bboxes[i][3])
        theta = float(bboxes[i][4])
        w = float(bboxes[i][5])
        l = float(bboxes[i][6])
        h = float(bboxes[i][7])
        vertices.append(convert_to_8corners(theta,x,y,z,l,w,h))
    for i in range(len(bboxes)):
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(vertices[i])
        line_set.lines = o3d.utility.Vector2iVector(lines)
        if int(bboxes[i][0]) == 1 :
            line_set.colors = o3d.utility.Vector3dVector(colors_1)
        elif int(bboxes[i][0]) == 0 :
            line_set.colors = o3d.utility.Vector3dVector(colors_0)
        label_content.append(line_set)
    return label_content

def open_label_file():
    global label_content
    global label_path
    filetypes = (
        ('.TXT', '*.txt'),
        ('.JSON', '*.json'),
        ('All files', '*.*')
    )
    filename = filedialog.askopenfilename(filetypes=filetypes)
    label_path = os.path.realpath(filename)
    l_ext=pathlib.Path(label_path).suffix
    if l_ext == '.txt':
        label_content = label_handler(label_path)
    elif l_ext == '.json':
        bboxes = json_to_kitti(label_path)
        label_content = json_handler(bboxes)
    else:
        messagebox.showerror("ERROR", "INVALID FILE EXTENSION")
    label_entry.delete(0, END)
    label_entry.insert(0, label_path)
    

root = Tk()
root.title('Vizty')
root.geometry('950x250')

mf = Frame(root)
mf.pack()

f1 = Frame(mf, width=600, height=250)
f1.pack(fill=X)
f2 = Frame(mf, width=600, height=250)
f2.pack(fill=X)
f3 = Frame(mf, width=600, height=250)
f3.pack()

file_path = StringVar
label_path = StringVar

Label(f1,text="Select Your File ").grid(row=0, column=0, sticky='e')
pc_entry = Entry(f1, width=50, textvariable=file_path)
pc_entry.grid(row=0,column=1,padx=2,pady=2,sticky='we',columnspan=25)
Button(f1, text="Browse", command=open_pc_file).grid(row=0, column=27, sticky='ew', padx=8, pady=4)

Label(f2,text="Select Your Labels ").grid(row=1, column=0, sticky='e')
label_entry = Entry(f2, width=50, textvariable=label_path)
label_entry.grid(row=1,column=1,padx=2,pady=2,sticky='we',columnspan=25)
Button(f2, text="Browse", command=open_label_file).grid(row=1, column=27, sticky='ew', padx=8, pady=4)

Button(f3, text="Visualize", width=32, command=lambda: visualize_pc_file(pc_content,label_content)).grid(sticky='ew', padx=10, pady=10)

root.mainloop()
