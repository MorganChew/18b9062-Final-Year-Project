# ikpy_chain.py
import ikpy.chain
import numpy as np

def create_ikpy_chain(urdf_file_path):
    ik_solver = ikpy.chain.Chain.from_urdf_file(urdf_file_path)
    return ik_solver
