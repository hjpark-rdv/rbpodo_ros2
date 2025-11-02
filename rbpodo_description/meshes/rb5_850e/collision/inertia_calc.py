import numpy as np
import struct  # 바이너리 파싱용

def compute_inertia_from_stl(stl_file_path, density=1.0):
    with open(stl_file_path, 'rb') as f:
        # STL 헤더 (80바이트) 스킵
        f.read(80)
        # 삼각형 개수
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        vertices = []
        faces = []
        
        for _ in range(num_triangles):
            # 노멀 벡터 스킵 (12바이트)
            f.read(12)
            # 세 정점 (각 12바이트: x,y,z float)
            v1 = struct.unpack('<fff', f.read(12))
            v2 = struct.unpack('<fff', f.read(12))
            v3 = struct.unpack('<fff', f.read(12))
            vertices.extend([v1, v2, v3])
            faces.append(len(vertices) - 3)  # 인덱스 대신 카운트 사용 (중복 제거 안 함)
            # 속성 바이트 스킵
            f.read(2)
    
    # 중복 제거 없이 vertices 배열 (N*3)
    vertices = np.array(vertices)
    # Faces: 각 삼각형의 시작 인덱스 (0,3,6,... 가정, 실제로는 flat)
    faces = np.arange(num_triangles * 3).reshape(num_triangles, 3)
    
    # 중심으로 이동
    center = np.mean(vertices, axis=0)
    vertices_centered = vertices - center
    
    # Inertia tensor 계산 (쉘 근사, dm = density * area)
    I = np.zeros((3, 3))
    total_mass = 0
    
    for i in range(num_triangles):
        tri_indices = faces[i]
        v1, v2, v3 = vertices_centered[tri_indices]
        
        # 삼각형 면적
        cross = np.cross(v2 - v1, v3 - v1)
        area = 0.5 * np.linalg.norm(cross)
        dm = density * area  # 두께 1 가정
        total_mass += dm
        
        # 각 정점 기여 (1/3 분배)
        for v in [v1, v2, v3]:
            I[0, 0] += (dm / 3) * (v[1]**2 + v[2]**2)  # Ixx
            I[1, 1] += (dm / 3) * (v[0]**2 + v[2]**2)  # Iyy
            I[2, 2] += (dm / 3) * (v[0]**2 + v[1]**2)  # Izz
            I[0, 1] -= (dm / 3) * v[0] * v[1]         # off-diagonal
            I[0, 2] -= (dm / 3) * v[0] * v[2]
            I[1, 2] -= (dm / 3) * v[1] * v[2]
    
    # Symmetric
    I = (I + I.T) / 2
    
    # Principal moments (대각)
    ixx, iyy, izz = I[0,0], I[1,1], I[2,2]

    # Symmetric 후 출력
    print("Inertia Tensor:")
    print(f"  xx: {I[0,0]:.6f}")
    print(f"  xy: {I[0,1]:.6f}")
    print(f"  xz: {I[0,2]:.6f}")
    print(f"  yy: {I[1,1]:.6f}")
    print(f"  yz: {I[1,2]:.6f}")
    print(f"  zz: {I[2,2]:.6f}")
    print(f"(density={density}, Total mass: {total_mass:.6f})")

    return f"계산된 Inertia (density={density}): Ixx={ixx:.4f}, Iyy={iyy:.4f}, Izz={izz:.4f} (Total mass: {total_mass:.4f})"

# 사용 예시
result = compute_inertia_from_stl('link0.stl', density=1.0)
print(result)