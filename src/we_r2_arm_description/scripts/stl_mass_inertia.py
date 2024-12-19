import trimesh


def main():
    try:
        mesh_name = input("The name of the stl file: ")
        density = input('The density(kg/m³): ')

        mesh = trimesh.load_mesh("../meshes/" + mesh_name + '.stl')

        if not density:
            density = 190 # kg/m³ (PLA - 15%)
            print('The density by default -', density)

        volume = mesh.volume
        mass = volume * int(density)

        inertia = mesh.moment_inertia
        ixx, iyy, izz = inertia[0][0], inertia[1][1], inertia[2][2]
        ixy, ixz, iyz = inertia[0][1], inertia[0][2], inertia[1][2]

        print('mass:', mass)

        print('ixx:', ixx)
        print('iyy:', iyy)
        print('izz:', izz)
        print('ixy:', ixy)
        print('ixz:', ixz)
        print('iyz:', iyz)
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()