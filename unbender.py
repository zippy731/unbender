bl_info = {
    "name": "Blender Unbender",
    "author": "Chris Allen",
    "version": (0, 9, 0),
    "blender": (4, 1, 0),
    "location": "View3D > Sidebar > Unbender",
    "description": "Unfold 3D meshes with animated shapekeys",
    "warning": "",
    "doc_url": "",
    "category": "Object",
}

import bpy
import bmesh
import math
import mathutils
from mathutils import Vector
from collections import deque, defaultdict

def print_header(text):
    """Print a nicely formatted header for sections."""
    print(f"\n{'=' * 50}")
    print(f"  {text}")
    print(f"{'=' * 50}")

def print_subheader(text):
    """Print a nicely formatted subheader."""
    print(f"\n{'-' * 40}")
    print(f"  {text}")
    print(f"{'-' * 40}")

def get_face_center(mesh, face):
    """Calculate the center of a face."""
    center = Vector((0, 0, 0))
    for v_idx in face.vertices:
        center += mesh.vertices[v_idx].co
    return center / len(face.vertices)

def pick_origin_face(obj, mesh):
    """
    Select the face with:
    0) verts marked 'origin' (if a vertex group named 'origin' exists and contains all verts of a face)
    1) lowest Z-center
    2) if tie, largest area
    3) if tie, most edges (verts)
    4) if tie, lowest face index
    """
    print_subheader("SELECTING ORIGIN FACE")

    # Check if there's a vertex group named 'origin'
    origin_group = None
    if obj.vertex_groups:
        for vg in obj.vertex_groups:
            if vg.name.lower() == 'origin':
                origin_group = vg
                print(f"  Found 'origin' vertex group (ID: {vg.index})")
                break
    
    # If we found an origin group, check if any face has all its vertices in the group
    if origin_group:
        print(f"  Checking for faces with all vertices in 'origin' group")
        
        # Create a set of vertices in the origin group
        origin_verts = set()
        for v_idx in range(len(mesh.vertices)):
            try:
                # Try to get the weight of this vertex in the origin group
                # If it succeeds, the vertex is in the group
                weight = origin_group.weight(v_idx)
                # Only include vertices with significant weight
                if weight > 0.5:
                    origin_verts.add(v_idx)
            except RuntimeError:
                # This vertex is not in the group
                pass
        
        print(f"  Found {len(origin_verts)} vertices in 'origin' group")
        
        # Check each face to see if all its vertices are in the origin group
        candidate_faces = []
        for f in mesh.polygons:
            face_verts = set(f.vertices)
            if face_verts.issubset(origin_verts) and len(face_verts) > 0:
                candidate_faces.append(f)
                print(f"  Found face {f.index} with all {len(face_verts)} vertices in 'origin' group")
        
        # If we have multiple candidate faces, pick the one with the most vertices
        if len(candidate_faces) > 0:
            if len(candidate_faces) > 1:
                print(f"  Found {len(candidate_faces)} faces with all vertices in 'origin' group")
                # Sort by number of vertices (descending), then by lowest Z-center (ascending)
                candidate_faces.sort(key=lambda f: (-len(f.vertices), get_face_center(mesh, f).z))
                best_face = candidate_faces[0]
                print(f"  Selected face {best_face.index} with {len(best_face.vertices)} vertices as origin face")
            else:
                best_face = candidate_faces[0]
                print(f"  Selected origin face: {best_face.index} (from vertex group)")
            
            return best_face.index
    
    # If no face was found with all vertices in the origin group, use the default method
    print("  No face found with all vertices in 'origin' group, using default selection method")
    
    best_idx = None
    best_data = None

    for f in mesh.polygons:
        center = get_face_center(mesh, f)
        z_center = center.z
        area = f.area
        num_verts = len(f.vertices)
        compare_tuple = (z_center, -area, -num_verts, f.index)

        if best_idx is None or compare_tuple < best_data:
            best_idx = f.index
            best_data = compare_tuple
            print(f"  New best face: {f.index} with z={z_center:.4f}, area={area:.4f}, verts={num_verts}")

    print(f"  Selected origin face: {best_idx}")
    return best_idx

def build_mesh_connectivity(mesh):
    """
    Build the mesh connectivity information including:
    - face adjacency (which faces are connected to each face)
    - edge to faces mapping (which faces share each edge)
    - each face's edges
    """
    print_subheader("BUILDING MESH CONNECTIVITY")

    # Map edges to faces
    edge_to_faces = defaultdict(list)
    for f in mesh.polygons:
        for edge_key in f.edge_keys:
            # Sort edge keys for consistency
            sorted_edge = tuple(sorted(edge_key))
            edge_to_faces[sorted_edge].append(f.index)

    # Build face adjacency
    face_adjacency = defaultdict(set)
    for edge_key, faces in edge_to_faces.items():
        if len(faces) == 1:
            # This is an outer edge (boundary)
            continue

        for i in range(len(faces)):
            for j in range(i+1, len(faces)):
                face_adjacency[faces[i]].add(faces[j])
                face_adjacency[faces[j]].add(faces[i])

    # Build face to edges mapping
    face_to_edges = {}
    for f in mesh.polygons:
        face_to_edges[f.index] = [tuple(sorted(ek)) for ek in f.edge_keys]

    print(f"  Total faces: {len(mesh.polygons)}")
    print(f"  Total unique edges: {len(edge_to_faces)}")
    print(f"  Found {sum(1 for faces in edge_to_faces.values() if len(faces) == 1)} outer edges")

    # Print a few sample face adjacencies
    sample_count = min(5, len(face_adjacency))
    print(f"  Sample adjacencies (showing {sample_count}):")
    for i, (face_idx, neighbors) in enumerate(list(face_adjacency.items())[:sample_count]):
        print(f"   Face {face_idx}: connected to {neighbors}")

    return face_adjacency, edge_to_faces, face_to_edges

def find_shared_edge(mesh, face1, face2, face_to_edges):
    """Find the shared edge between two faces."""
    edges1 = set(face_to_edges[face1])
    edges2 = set(face_to_edges[face2])
    shared = edges1.intersection(edges2)

    if shared:
        return next(iter(shared))  # Return the first shared edge
    return None

def initialize_paths(mesh, origin_face_idx, face_adjacency, face_to_edges):
    """
    Initialize one path for each edge of the origin face.
    Each path starts with just the origin face.
    """
    print_subheader("INITIALIZING PATHS")

    # Get all edges of the origin face
    origin_edges = face_to_edges[origin_face_idx]

    # Initialize paths
    paths = []
    for i, edge_key in enumerate(origin_edges):
        path = {
            "id": i,
            "origin_edge": edge_key,
            "faces": {
                origin_face_idx: {
                    "level": 0,
                    "parent": None,
                    "connecting_edge": None,
                    "children": []
                }
            },
            "edges": set(),  # Edges used in this path
            "face_indices": {origin_face_idx}, # Initialize face_indices set, starting with origin_face_idx
            "highest_level": 0,
            "faces_at_highest_level": {origin_face_idx},
            "max_level": 0,
            "initial_unfold_angle": 0.0,
            "direction_reason_initial_angle": "Origin Face",
            "levels": {  # <--- Ensure "levels" key is initialized correctly as a dictionary
                0: {origin_face_idx} # <--- Level 0 should contain origin_face_idx as a set
            },
            "origin_face_idx": origin_face_idx, # <--- ADD THIS LINE INSIDE DICT INITIALIZATION
        }
        paths.append(path)
        print(f"  Initialized path {i} for origin edge {edge_key}")

    return paths

def is_edge_adjacent_to_path_edges(edge_key, face_idx, path, face_to_edges):
    """
    Check if an edge shares any vertices with edges already in the path
    that are connected to this face.
    """
    # Get all vertices of the candidate edge
    v1, v2 = edge_key
    edge_verts = {v1, v2}

    # Check all edges currently in the path that belong to this face
    for path_edge in path["edges"]:
        # If the edge is part of this face
        if path_edge in face_to_edges[face_idx]:
            # Get vertices of path edge
            pv1, pv2 = path_edge
            path_verts = {pv1, pv2}

            # If they share any vertices, they're adjacent
            if edge_verts.intersection(path_verts):
                return True

    return False

def select_best_edge_for_face(face_idx, available_edges, path, face_to_edges, mesh):
    """
    Select the best edge to traverse from a face based on the criteria:
    1. Prefer edges not adjacent to current path edges
    2. If face is a quad, prefer edge opposite to entry edge
    3. For pentagon/hexagon, skip nearest edge and choose next one
    4. If no clear choice, use lowest index edge
    """
    # Get all edges of this face that are still available
    face_edges = face_to_edges[face_idx]
    candidate_edges = [e for e in face_edges if e in available_edges]

    if not candidate_edges:
        return None

    # Check if this is a quad face (4 edges)
    if len(face_edges) == 4:
        # Find the entry edge (the edge connecting to parent)
        parent_face = path["faces"][face_idx]["parent"]
        if parent_face is not None:
            entry_edge = path["faces"][face_idx]["connecting_edge"]

            # Find the opposite edge
            for e in candidate_edges:
                # A simple way to check for "opposite" - no shared vertices
                e_verts = set(e)
                entry_verts = set(entry_edge)
                if not e_verts.intersection(entry_verts):
                    return e

    # For all faces, first try to find edges not adjacent to current path edges
    non_adjacent_edges = []
    for e in candidate_edges:
        if not is_edge_adjacent_to_path_edges(e, face_idx, path, face_to_edges):
            non_adjacent_edges.append(e)

    if non_adjacent_edges:
        # Sort by edge index and return the lowest one
        sorted_edges = sorted(non_adjacent_edges, key=lambda e: (e[0], e[1]))
        return sorted_edges[0]

    # If no non-adjacent edges, just pick the lowest index edge
    sorted_edges = sorted(candidate_edges, key=lambda e: (e[0], e[1]))
    return sorted_edges[0]

def select_face_with_fewest_edges_in_path(faces, path, face_to_edges):
    """
    From a list of faces, select the one that has the fewest edges
    already in the path. Break ties using lowest face index.
    """
    face_scores = []

    for face_idx in faces:
        # Count how many edges of this face are already in the path
        face_edges = face_to_edges[face_idx]
        used_edges = sum(1 for e in face_edges if e in path["edges"])
        face_scores.append((used_edges, face_idx))

    # Sort by used edges, then by face index
    face_scores.sort()
    return face_scores[0][1] if face_scores else None

def grow_path_one_step(path, available_faces, available_edges, face_adjacency, edge_to_faces, face_to_edges, mesh, copy_obj, split_edges, outer_edges, frozen_edges=None): # Added frozen_edges parameter
    """Grows a single path by one face, round-robin style, with improved edge management."""
    if frozen_edges is None:
        frozen_edges = set()
        
    current_level = path["max_level"]
    faces_at_current_level = path["levels"][current_level]
    faces_at_next_level = set()
    path_grown = False

    print(f" Path {path['id']} turn:")
    print(f" Path {path['id']}: Growing from level {current_level}")
    print(f"  Faces at this level: {faces_at_current_level}")

    for current_face_idx in faces_at_current_level:
        print(f"  Selected face {current_face_idx} for traversal")
        current_face_edges = face_to_edges[current_face_idx]
        
        # --- INSERT DEBUG PRINTS HERE ---
        print(f"  [EDGE-AVAIL-DEBUG] Before intersection - Available edges (count): {len(available_edges)}")

        available_face_edges = list(available_edges.intersection(current_face_edges)) # Edges of current face that are still available

        print(f"  [EDGE-AVAIL-DEBUG] After intersection - Traversable edges: {available_face_edges}") 
        print(f"  Available edges on face {current_face_idx} for traversal: {available_face_edges}")
        # --- END INSERT DEBUG PRINTS ---

        if not available_face_edges:
            print(f"  No available edges on face {current_face_idx} to traverse.")
            continue # No edges to traverse for this face

        # Filter out boundary edges (edges that only connect to one face)
        valid_edges = []
        for edge in available_face_edges:
            if len(edge_to_faces[edge]) == 2:
                valid_edges.append(edge)
            else:
                print(f"  Edge {edge} is a boundary edge (connects to only {len(edge_to_faces[edge])} face), skipping.")
        
        if not valid_edges:
            print(f"  No valid non-boundary edges on face {current_face_idx} to traverse.")
            continue
        
        # Find 'best' edge to traverse with new priority:
        # 1. Frozen edges (highest priority)
        # 2. Outer edges
        # 3. Non-split edges
        # 4. Any available edge
        best_edge = None
        best_edge_type = None
        
        # First priority: Frozen edges
        frozen_face_edges = [e for e in valid_edges if e in frozen_edges]
        if frozen_face_edges:
            best_edge = frozen_face_edges[0]
            best_edge_type = 'frozen'
        # Second priority: Outer edges
        elif any(e in outer_edges for e in valid_edges):
            outer_face_edges = [e for e in valid_edges if e in outer_edges]
            best_edge = outer_face_edges[0]
            best_edge_type = 'outer'
        # Third priority: Non-split edges
        elif any(e not in split_edges for e in valid_edges):
            non_split_face_edges = [e for e in valid_edges if e not in split_edges]
            best_edge = non_split_face_edges[0]
            best_edge_type = 'non-split'
        # Last resort: Any valid edge
        elif valid_edges:
            best_edge = valid_edges[0]
            best_edge_type = 'any'

        if best_edge:
            print(f"  Selected edge {best_edge} for traversal (type: {best_edge_type})")

            # Determine next face index - now safe because we've filtered for edges with exactly 2 faces
            new_face_idx = edge_to_faces[best_edge][1] if edge_to_faces[best_edge][0] == current_face_idx else edge_to_faces[best_edge][0]

            print(f"   [DEBUG-ORIGIN-FACE-KEY] Before check - Path ID: {path['id']},  Keys in path: {path.keys()}")
            print(f"   [DEBUG-ORIGIN-FACE-KEY] Origin Face Index from Path: {path.get('origin_face_idx', 'KeyError or Missing')}")

            # Check if we are not traversing back to origin face
            if new_face_idx == path["origin_face_idx"]:
                print(f"  Avoided traversing back to origin face {new_face_idx} via edge {best_edge}")
                available_edges.discard(best_edge) # Make this specific edge unavailable for this path
                continue # Skip to next available edge or face

            if new_face_idx in path["face_indices"]:
                print(f"  Avoided revisiting face {new_face_idx} via edge {best_edge}")
                available_edges.discard(best_edge) # Make this specific edge unavailable
                continue

            # LOGIC REFINEMENT #3: Check if the target face is still available
            if new_face_idx not in available_faces:
                print(f"  Face {new_face_idx} is no longer available (already unfolded by another path)")
                available_edges.discard(best_edge) # Make this specific edge unavailable
                continue

            # Update path data with new face and edge
            if (current_level + 1) not in path["levels"]:
                path["levels"][current_level + 1] = set()
            path["levels"][current_level + 1].add(new_face_idx)
            path["max_level"] += 1
            path["face_indices"].add(new_face_idx)
            path["faces"][new_face_idx] = {
                "parent": current_face_idx,
                "connecting_edge": best_edge,
                "children": [],
                "level": current_level + 1,
                "initial_unfold_angle": None,  # Initialize as None, will be calculated after splitting
                "direction_reason_initial_angle": None  # Initialize as None, will be calculated after splitting
            }
            path["faces"][current_face_idx]["children"].append(new_face_idx)

            faces_at_next_level.add(new_face_idx)
            path["edges"].add(best_edge)
            
            # LOGIC REFINEMENT #3: Only remove the specific face and edge we used
            available_faces.remove(new_face_idx)  # Remove the face we moved TO
            available_edges.discard(best_edge)   # Only remove the specific edge we used
            
            # LOGIC REFINEMENT #3: No longer removing ALL edges of the face we moved from
            # OLD CODE: available_edges.difference_update(current_face_edges)

            path_grown = True
        else:
            print(f"  No suitable edge found to traverse from face {current_face_idx} this round.")

    return path_grown, faces_at_next_level, available_faces, available_edges, split_edges, outer_edges

def collect_subtree_vertices(mesh, path, face_idx):
    """Return vertex indices for face_idx + all descendants."""
    stack = [face_idx]
    all_verts = set()
    print(f"  [CSV-DEBUG] Starting vertex collection for face {face_idx} subtree.")
    while stack:
        f = stack.pop()
        face_vertex_indices = list(mesh.polygons[f].vertices)
        print(f"  [CSV-DEBUG]   Face {f} (polygon index): Vertices (vertex indices) = {face_vertex_indices}, Children = {path['faces'][f]['children']}")
        for v in face_vertex_indices:
            all_verts.add(v)
        for c in path["faces"][f]["children"]:
            stack.append(c)
    collected_vert_indices = list(all_verts)
    print(f"  [CSV-DEBUG] Total collected verts for face {face_idx} subtree: {collected_vert_indices}")
    return collected_vert_indices

def edge_verts_positions(obj, edge_key): # Changed to accept 'obj'
    """Get 3D positions of an edge's vertices using obj.data.vertices."""
    vA_idx, vB_idx = edge_key
    return (obj.data.vertices[vA_idx].co.copy(), obj.data.vertices[vB_idx].co.copy()) # Use obj.data.vertices

def rotate_face_center(face_center, hinge_vA_co, rot_mat):
    """Rotate a face center point around hinge vertex A using the given rotation matrix."""
    local_center = face_center - hinge_vA_co
    rotated_center = rot_mat @ local_center
    final_center = rotated_center + hinge_vA_co
    return final_center

def find_correct_hinge_vertices(mesh, parent_face_idx, child_face_idx):
    """
    Find the correct shared edge vertices between parent and child faces.
    
    Args:
        mesh: The mesh data
        parent_face_idx: Index of parent face
        child_face_idx: Index of child face
        
    Returns:
        Tuple of (vertex_A_idx, vertex_B_idx) or None if no shared edge found
    """
    parent_verts = set(mesh.polygons[parent_face_idx].vertices)
    child_verts = set(mesh.polygons[child_face_idx].vertices)
    
    # Find shared vertices between the two faces
    shared_verts = parent_verts.intersection(child_verts)
    
    # We need exactly 2 vertices for a valid hinge
    if len(shared_verts) != 2:
        print(f"   WARNING: Found {len(shared_verts)} shared vertices, expected 2 for a valid hinge!")
        print(f"   Shared vertices: {shared_verts}")
        if len(shared_verts) < 2:
            return None
    
    # Convert to a list for indexing
    shared_verts = list(shared_verts)
    return (shared_verts[0], shared_verts[1])

def apply_hierarchical_rotations(obj, mesh, path, origin_face_idx, fraction, shape_key):
    """
    Apply rotations using a strict level-by-level approach to ensure correct
    hierarchical transformation ordering.
    """
    print_subheader(f"APPLYING HIERARCHICAL ROTATIONS FOR PATH {path['id']} AT {int(fraction*100)}%")
    
    # Start with Basis positions
    basis_key = obj.data.shape_keys.key_blocks["Basis"]
    for v_idx in range(len(mesh.vertices)):
        shape_key.data[v_idx].co = basis_key.data[v_idx].co.copy()
    
    # Create a vertex position tracker dictionary
    vertex_positions = {}
    for v_idx in range(len(mesh.vertices)):
        vertex_positions[v_idx] = basis_key.data[v_idx].co.copy()
    
    # First, organize all faces by level
    faces_by_level = {}
    for face_idx, face_data in path["faces"].items():
        level = face_data["level"]
        if level not in faces_by_level:
            faces_by_level[level] = []
        faces_by_level[level].append(face_idx)
    
    max_level = max(faces_by_level.keys()) if faces_by_level else 0
    print(f"  Path has {max_level+1} levels (0 to {max_level})")
    
    # Process each level from top to bottom
    for level in range(1, max_level + 1):  # Skip level 0 (origin face)
        if level not in faces_by_level:
            continue
            
        print(f"  Processing level {level}:")
        
        # Process all faces at this level
        for face_idx in faces_by_level[level]:
            face_data = path["faces"][face_idx]
            parent_idx = face_data["parent"]
            
            if parent_idx is None:
                print(f"  ERROR: Face {face_idx}: No parent found, skipping")
                continue
                
            # Get stored connecting edge and verify it
            connecting_edge = face_data["connecting_edge"]
            if connecting_edge is None:
                print(f"  ERROR: Face {face_idx}: No connecting edge found, skipping")
                continue
                
            # Verify the connecting edge is actually shared between parent and child
            parent_verts = set(mesh.polygons[parent_idx].vertices)
            child_verts = set(mesh.polygons[face_idx].vertices)
            
            vA_idx, vB_idx = connecting_edge
            if vA_idx not in parent_verts or vA_idx not in child_verts or vB_idx not in parent_verts or vB_idx not in child_verts:
                print(f"  CRITICAL ERROR: Connecting edge {connecting_edge} is not valid for faces {parent_idx} and {face_idx}")
                print(f"  Parent vertices: {parent_verts}")
                print(f"  Child vertices: {child_verts}")
                print(f"  Skipping rotation for this face")
                continue
            
            # Get angle to use
            initial_angle = face_data["initial_unfold_angle"]
            if initial_angle is None:
                print(f"  ERROR: Face {face_idx} has no initial unfold angle, skipping rotation")
                continue
                
            use_angle = initial_angle * fraction
            
            # CRITICAL: Use CURRENT vertex positions for hinge points, not original positions
            vA_co = vertex_positions[vA_idx]
            vB_co = vertex_positions[vB_idx]
            
            # Calculate the rotation axis vector
            axis_vec = (vB_co - vA_co).normalized()
            
            print(f"  Rotating face {face_idx} (child of {parent_idx}) around hinge {connecting_edge}")
            print(f"  Hinge points: A={vA_co}, B={vB_co}")
            print(f"  Rotation axis: {axis_vec}")
            print(f"  Angle: {math.degrees(use_angle):.2f} degrees (from stored angle: {math.degrees(initial_angle):.2f})")
            
            # Get all vertices in this face's subtree
            subtree_verts = collect_subtree_vertices(mesh, path, face_idx)
            print(f"  Subtree contains {len(subtree_verts)} vertices")
            
            # Create rotation matrix
            rot_mat = mathutils.Matrix.Rotation(use_angle, 4, axis_vec)
            
            # Apply rotation to all vertices in the subtree
            for v_idx in subtree_verts:
                # Start with CURRENT position
                current_pos = vertex_positions[v_idx]
                
                # Make position local to hinge point A
                local_pos = current_pos - vA_co
                
                # Apply rotation
                rotated_pos = rot_mat @ local_pos
                
                # Convert back to global coordinates
                final_pos = rotated_pos + vA_co
                
                # Update both the tracking dictionary and shape key
                vertex_positions[v_idx] = final_pos
                shape_key.data[v_idx].co = final_pos
                
                # Debug output for some vertices
                if v_idx in mesh.polygons[face_idx].vertices:
                    print(f"    Vertex {v_idx}: {current_pos} -> {final_pos}")
    
    print(f"  Completed hierarchical rotations for {shape_key.name}")

def setup_sequential_drivers(obj, path_id, fractions):
    """
    Set up drivers to make shape keys activate in sequence.
    Creates both individual path controls and a master control for all paths.
    """
    shape_keys = obj.data.shape_keys
    if not shape_keys:
        return
    
    # We'll use a single control slider for the entire sequence
    control_name = f"Path_{path_id:02d}_Control"
    
    # Add custom property to object if it doesn't exist
    if control_name not in obj:
        obj[control_name] = 0.0
        # Make it animatable
        obj.id_properties_ui(control_name).update(min=0.0, max=1.0)
        print(f"  Added control property {control_name} to object")
    
    # Add master control property if it doesn't exist yet
    master_control_name = "00_Master_Unfold_Control"
    if master_control_name not in obj:
        obj[master_control_name] = 0.0
        # Make it animatable
        obj.id_properties_ui(master_control_name).update(min=0.0, max=1.0)
        print(f"  Added master control property {master_control_name} to object")
    
    # Set up drivers for each shape key in the sequence
    for i, frac in enumerate(fractions):
        # Use the new hyphen-based naming format
        key_name = f"Path-{path_id:02d}-{int(frac*100)}%"
        if key_name in shape_keys.key_blocks:
            shape_key = shape_keys.key_blocks[key_name]
            
            # Create new driver
            fcurve = shape_key.driver_add("value")
            driver = fcurve.driver
            driver.type = 'SCRIPTED'
            
            # Add variable for the path control property
            var1 = driver.variables.new()
            var1.name = "pathcontrol"
            var1.type = 'SINGLE_PROP'
            var1.targets[0].id = obj
            var1.targets[0].data_path = f'["{control_name}"]'
            
            # Add variable for the master control property
            var2 = driver.variables.new()
            var2.name = "mastercontrol"
            var2.type = 'SINGLE_PROP'
            var2.targets[0].id = obj
            var2.targets[0].data_path = f'["{master_control_name}"]'
            
            # Calculate the range this shape key should be active in
            lower_bound = 0.0 if i == 0 else fractions[i-1]
            upper_bound = frac
            range_size = upper_bound - lower_bound
            
            # Create expression that activates this key in its range for EITHER control
            if range_size > 0:
                # This formula allows either control to drive the shape key
                # It maps each control (0-1) to this key's range (0-1)
                driver_expr = f"max(0, min(1, max((pathcontrol - {lower_bound:.2f}), (mastercontrol - {lower_bound:.2f})) * {1/range_size:.2f}))"
                driver.expression = driver_expr
                print(f"  Set driver for {key_name}: active from {lower_bound:.2f} to {upper_bound:.2f}")
                print(f"  DEBUG: Driver expression: {driver_expr}")
            else:
                print(f"  Warning: Cannot set driver for {key_name} - range size is 0")

def create_shape_keys_for_path(obj, mesh, path, origin_face_idx):
    """Create shape keys for unfolding animation with corrected sequential handling."""
    print_subheader(f"CREATING SHAPE KEYS FOR PATH {path['id']}")

    # Skip paths with no additional faces
    if len(path["faces"]) <= 1:
        print("  Path has no additional faces, skipping shape keys")
        return

    # Define fractions - we'll create absolute shape keys
    fractions = [0.25, 0.50, 0.75, 1.00]
    
    # Ensure all existing shape keys are at value 0
    reset_shape_key_values(obj)
    
    # Store the original basis shape key
    basis_key = obj.data.shape_keys.key_blocks["Basis"]
    
    # Create each shape key as a relative change from the previous key
    first_key = None
    previous_key = None
    
    # Store the initial vertex positions from the basis
    initial_positions = {}
    for v_idx in range(len(mesh.vertices)):
        initial_positions[v_idx] = basis_key.data[v_idx].co.copy()
    
    # Create a vertex position tracker to maintain state between shape keys
    vertex_positions = {}
    for v_idx in range(len(mesh.vertices)):
        vertex_positions[v_idx] = initial_positions[v_idx].copy()
    
    for frac in fractions:
        # Use hyphen-based naming for consistency with level-based shape keys
        sk_name = f"Path-{path['id']:02d}-{int(frac*100)}%"
        print(f"  Creating shape key: {sk_name}")
        
        # Create new shape key from mix (to build on previous keys)
        shape_key = obj.shape_key_add(name=sk_name, from_mix=True)
        
        # Store reference to first key
        if first_key is None:
            first_key = shape_key
        
        # Apply the absolute rotation for this fraction
        apply_hierarchical_rotations(obj, mesh, path, origin_face_idx, frac, shape_key)
        print(f"  Applied full {int(frac*100)}% rotation to {sk_name}")
        
        # Set relative key relationship
        if previous_key:
            # This key is relative to the previous key
            shape_key.relative_key = previous_key
            print(f"  Set {sk_name} relative to {previous_key.name}")
        
        # Store for next iteration
        previous_key = shape_key
    
    # Set up driver for automatic sequence
    if len(fractions) > 0 and first_key:
        setup_sequential_drivers(obj, path['id'], fractions)
    
    # Reset all shape keys to 0 before returning
    reset_shape_key_values(obj)
    print(f"  Completed all shape keys for path {path['id']}")

def print_path_summary(path):
    """Print a summary of a path's structure."""
    print(f"  Path {path['id']} summary:")
    print(f"   Origin edge: {path['origin_edge']}")
    print(f"   Total faces: {len(path['faces'])}")
    print(f"   Total edges: {len(path['edges'])}")
    print(f"   Max hierarchy level: {path['highest_level']}")

    # Count faces at each level
    level_counts = {}
    for face_idx, face_data in path["faces"].items():
        level = face_data["level"]
        level_counts[level] = level_counts.get(level, 0) + 1

    print(f"   Faces by level: {level_counts}")

    # Print hierarchy tree
    if len(path["faces"]) > 1:
        print(f"   Hierarchy tree:")

        def print_tree(face_idx, depth=0):
            face_data = path["faces"][face_idx]
            indent = "     " + "  " * depth
            edge_info = ""
            if face_data["connecting_edge"]:
                edge_info = f" via edge {face_data['connecting_edge']}"
            print(f"{indent}Face {face_idx} (level {face_data['level']}){edge_info}")

            for child in face_data["children"]:
                print_tree(child, depth + 1)

        # Find origin face (level 0)
        for face_idx, face_data in path["faces"].items():
            if face_data["level"] == 0:
                print_tree(face_idx)
                break

def split_edges_and_duplicate_verts(mesh, all_fold_edges):
    """
    Split non-fold edges using bmesh.ops.split_edges.
    (Corrected for Blender 4.1 - use_duplicate=True removed, NO manual duplication)
    """
    print_subheader("SPLITTING EDGES AND DUPLICATING VERTS")

    initial_vertex_count = len(mesh.vertices)

    # Collect all fold edges from all paths (already done in main script)
    # all_fold_edges = set()
    # for path in paths:
    #     all_fold_edges.update(path["edges"])
    print(f"Total fold edges: {len(all_fold_edges)}")

    # Check if any edges have been processed
    if len(all_fold_edges) == 0:
        print("WARNING: No fold edges found! All edges will be split and verts duplicated.")
        print("Check the path building logic.")

    # Instead of selection and ops.edge_split, use BMesh for direct edge & vert splitting
    import bmesh

    # Make sure we're in object mode
    if bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    # Create BMesh from the mesh
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.edges.ensure_lookup_table()

    edges_to_split = []
    edge_keys_to_split = []

    for edge in bm.edges:
        v0, v1 = edge.verts[0].index, edge.verts[1].index
        edge_key = tuple(sorted((v0, v1)))
        if edge_key not in all_fold_edges:
            edges_to_split.append(edge)
            edge_keys_to_split.append(edge_key)

    split_count = len(edges_to_split)
    print(f"Found {split_count} BMesh edges to split and duplicate verts:")
    # for ek in edge_keys_to_split: # No longer printing all edge keys to avoid clutter
    #     print(f"  - Edge {ek}")

    # Only split and duplicate if we have edges to split and some fold edges exist
    if split_count > 0: # Removed the check for `len(all_fold_edges) > 0` to allow splitting all if no fold edges found.
        # 1. Split the edges using BMesh's edge splitting
        split_result = bmesh.ops.split_edges(bm, edges=edges_to_split)  # use_duplicate=True REMOVED - THIS IS CORRECT!
        split_edges_new = split_result['edges']

        final_vertex_count = len(mesh.vertices)
        vertices_added = final_vertex_count - initial_vertex_count
        print(f"Split {split_count} edges (using bmesh.ops.split_edges), added {vertices_added} vertices.") # Adjusted print message
    else:
        print("No edges to split or duplicate.")


    # Update the mesh data from BMesh
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()

    # Explicitly ensure we're back in Object mode after BMesh operations
    if bpy.context.object.mode != 'OBJECT':
        print("  Switching back to Object Mode after BMesh operations")
        bpy.ops.object.mode_set(mode='OBJECT')

    print(f"Mesh verts after duplication: {len(mesh.vertices)}") # Debug: Check vertex count

def reset_shape_key_values(obj):
    """Reset all shape key values to 0 for the given object."""
    print_subheader("RESETTING SHAPE KEY VALUES")
    if obj.data.shape_keys:
        for key_block in obj.data.shape_keys.key_blocks:
            key_block.value = 0.0
            print(f"  Resetting shape key: {key_block.name} to value 0.0")
    else:
        print("  No shape keys found to reset.")

def calculate_post_split_angles(mesh, paths, copy_obj, frozen_edges=None):
    """
    Calculate and store unfolding angles for all paths AFTER edge splitting.
    This ensures angles are calculated using the final mesh topology.
    
    For edges with bevel weight > 0, the unfold angle is set to 0
    to maintain the folded angle during animation.
    
    Args:
        mesh: The mesh data after edge splitting
        paths: List of all paths
        copy_obj: The duplicated object with the split mesh
        frozen_edges: Set of edge keys (tuples of vertex indices) that should maintain their folded angle
                     (This parameter is kept for backward compatibility but not used)
    """
    print_subheader("CALCULATING POST-SPLIT UNFOLDING ANGLES")
    
    # Create a BMesh to directly access edge bevel weight data in the post-split mesh
    import bmesh
    
    # Make sure we're in object mode
    if bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    
    # Create BMesh from the mesh
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.edges.ensure_lookup_table()
    
    # Try to get the bevel weight layer
    try:
        bevel_layer = bm.edges.layers.float.get('bevel_weight_edge')
        if bevel_layer is None:
            bevel_layer = bm.edges.layers.float.get('bevel_weight')
            if bevel_layer is None:
                print("  No bevel weight layer found, no edges will be frozen")
                bevel_layer = None
    except Exception as e:
        print(f"  Warning: Could not access bevel weight layer: {e}")
        print("  No edges will be frozen")
        bevel_layer = None
    
    # Create a dictionary to map vertex pairs to BMesh edges
    edge_map = {}
    if bevel_layer is not None:
        for edge in bm.edges:
            v0, v1 = edge.verts
            # Store both orientations of the edge key for easier lookup
            edge_key1 = (v0.index, v1.index)
            edge_key2 = (v1.index, v0.index)
            edge_map[edge_key1] = edge
            edge_map[edge_key2] = edge
    
    # Process each path
    for path_idx, path in enumerate(paths):
        print(f"Processing angles for path {path_idx}:")
        
        # Skip the origin face (level 0)
        # Process each face by level, starting from level 1
        for level in range(1, path["max_level"] + 1):
            if level not in path["levels"]:
                continue
                
            print(f"  Processing level {level}:")
            
            # Process all faces at this level
            for face_idx in path["levels"][level]:
                face_data = path["faces"][face_idx]
                parent_idx = face_data["parent"]
                
                if parent_idx is None:
                    print(f"  ERROR: Face {face_idx} has no parent, skipping angle calculation")
                    continue
                    
                # Get stored connecting edge
                connecting_edge = face_data["connecting_edge"]
                if connecting_edge is None:
                    print(f"  ERROR: Face {face_idx} has no connecting edge, skipping angle calculation")
                    continue
                
                # Verify the connecting edge is actually shared between parent and child
                parent_verts = set(mesh.polygons[parent_idx].vertices)
                child_verts = set(mesh.polygons[face_idx].vertices)
                
                vA_idx, vB_idx = connecting_edge
                if vA_idx not in parent_verts or vA_idx not in child_verts or vB_idx not in parent_verts or vB_idx not in child_verts:
                    print(f"  ERROR: Connecting edge {connecting_edge} is not valid for faces {parent_idx} and {face_idx}")
                    # Find a valid connecting edge
                    hinge = find_correct_hinge_vertices(mesh, parent_idx, face_idx)
                    if hinge is None:
                        print(f"  CRITICAL ERROR: No valid hinge found between faces {parent_idx} and {face_idx}")
                        continue
                    
                    # Update the connecting edge in the path data
                    connecting_edge = hinge
                    face_data["connecting_edge"] = hinge
                    vA_idx, vB_idx = connecting_edge
                    print(f"  Updated connecting edge to: {connecting_edge}")
                
                # Check if this is a frozen edge by directly checking bevel weight
                should_freeze = False
                if bevel_layer is not None:
                    # Try both orientations of the edge
                    edge_key1 = (vA_idx, vB_idx)
                    edge_key2 = (vB_idx, vA_idx)
                    
                    if edge_key1 in edge_map:
                        bm_edge = edge_map[edge_key1]
                        bevel_value = bm_edge[bevel_layer]
                        should_freeze = bevel_value > 0
                    elif edge_key2 in edge_map:
                        bm_edge = edge_map[edge_key2]
                        bevel_value = bm_edge[bevel_layer]
                        should_freeze = bevel_value > 0
                
                if should_freeze:
                    # Set unfold angle to 0 for frozen edges
                    face_data["initial_unfold_angle"] = 0.0
                    face_data["direction_reason_initial_angle"] = "Frozen edge (bevel weight > 0)"
                    print(f"  [FROZEN-EDGE] Face {face_idx} (child of {parent_idx}): Edge {connecting_edge} is frozen, setting unfold angle to 0°")
                    continue
                
                # Get vertex positions for the hinge edge
                vA_co, vB_co = edge_verts_positions(copy_obj, connecting_edge)
                
                # Calculate angle using the direct normal-based method
                rotation_angle = calculate_rotation_angle_from_normals(
                    mesh.polygons[parent_idx], 
                    mesh.polygons[face_idx],
                    vA_co, vB_co
                )
                
                # Store the calculated angle
                face_data["initial_unfold_angle"] = rotation_angle
                face_data["direction_reason_initial_angle"] = "Direct normal calculation"
                
                print(f"  [POST-SPLIT-ANGLE] Face {face_idx} (child of {parent_idx}): Using angle: {math.degrees(rotation_angle):.4f}°")
    
    # Free the BMesh
    bm.free()
    
    print("Completed post-split angle calculations for all paths")

def calculate_rotation_angle_from_normals(parent_face, child_face, hinge_vA_co, hinge_vB_co):
    """
    Calculate the rotation angle needed to make two faces coplanar with normals pointing
    in the SAME direction (parallel) after unfolding.
    
    This function uses a direct mathematical approach:
    1. Project both face normals onto a plane perpendicular to the hinge edge
    2. Calculate the angle between these projections
    3. Determine the correct rotation direction using the cross product
    4. Invert the angle for unfolding (rather than folding)
    
    Args:
        parent_face: The parent face polygon
        child_face: The child face polygon
        hinge_vA_co: 3D position of first vertex of the hinge edge
        hinge_vB_co: 3D position of second vertex of the hinge edge
        
    Returns:
        float: The signed rotation angle in radians needed for unfolding
    """
    # Get face normals
    parent_normal = parent_face.normal.copy()
    child_normal = child_face.normal.copy()
    
    # Check if faces are already coplanar (normals are parallel or anti-parallel)
    # We use a small threshold to account for floating-point precision
    normal_dot = parent_normal.dot(child_normal)
    if abs(abs(normal_dot) - 1.0) < 1e-4:  # Normals are nearly parallel or anti-parallel
        print(f"  [COPLANAR-FACES] Detected coplanar faces: dot product = {normal_dot:.6f}")
        print(f"  [COPLANAR-FACES] Setting rotation angle to 0")
        return 0.0  # No rotation needed
    
    # Calculate the rotation axis (the hinge edge direction)
    hinge_vec = (hinge_vB_co - hinge_vA_co).normalized()
    
    # Project both normals onto a plane perpendicular to the hinge
    # This isolates the components of the normals that will be affected by rotation
    parent_proj = parent_normal - parent_normal.dot(hinge_vec) * hinge_vec
    child_proj = child_normal - child_normal.dot(hinge_vec) * hinge_vec
    
    # Check if either projection is too small (nearly zero)
    # This can happen when a face normal is nearly parallel to the hinge
    if parent_proj.length < 1e-6 or child_proj.length < 1e-6:
        print(f"  [DEGENERATE-CASE] Detected face normal nearly parallel to hinge")
        print(f"  [DEGENERATE-CASE] Parent projection length: {parent_proj.length:.6f}")
        print(f"  [DEGENERATE-CASE] Child projection length: {child_proj.length:.6f}")
        print(f"  [DEGENERATE-CASE] Setting rotation angle to 0")
        return 0.0  # Skip rotation for this degenerate case
    
    # Normalize the projections
    parent_proj.normalize()
    child_proj.normalize()
    
    # Calculate the angle between the projected normals
    # This is the exact angle needed to make the faces coplanar
    dot_product = parent_proj.dot(child_proj)
    
    # Handle numerical precision issues with dot product
    dot_product = max(min(dot_product, 1.0), -1.0)
    
    angle = math.acos(dot_product)
    
    # If angle is very small, consider faces already aligned
    if angle < 1e-4:  # Less than ~0.006 degrees
        print(f"  [SMALL-ANGLE] Detected very small angle: {math.degrees(angle):.6f}°")
        print(f"  [SMALL-ANGLE] Setting rotation angle to 0")
        return 0.0
    
    # Determine the sign of the rotation using the cross product
    # This tells us which direction to rotate to align the normals
    cross_product = parent_proj.cross(child_proj)
    sign = 1 if cross_product.dot(hinge_vec) > 0 else -1
    
    # Calculate the signed rotation angle
    rotation_angle = sign * angle

    # Invert the angle - we want to unfold, not fold
    # The calculated angle shows how much the faces are currently folded
    # To unfold, we need to rotate in the opposite direction
    rotation_angle = -rotation_angle
    
    # Debug output
    print(f"  [NORMAL-ANGLE-DEBUG] Parent normal: {parent_normal}, Child normal: {child_normal}")
    print(f"  [NORMAL-ANGLE-DEBUG] Hinge vector: {hinge_vec}")
    print(f"  [NORMAL-ANGLE-DEBUG] Projected parent: {parent_proj}, Projected child: {child_proj}")
    print(f"  [NORMAL-ANGLE-DEBUG] Dot product: {dot_product:.4f}, Angle between projections: {math.degrees(angle):.4f}°")
    print(f"  [NORMAL-ANGLE-DEBUG] Cross product: {cross_product}, Sign: {sign}")
    print(f"  [NORMAL-ANGLE-DEBUG] Final rotation angle: {math.degrees(rotation_angle):.4f}°")
    
    return rotation_angle

def identify_and_split_seam_edges(mesh, copy_obj):
    """
    Identify edges marked as seams and split them before pathfinding.
    This allows users to manually mark edges they want to be cut during unfolding.
    
    Edges with bevel weight > 0 are marked as "frozen" and will maintain their folded angle.
    If an edge has both seam and bevel weight, bevel weight takes precedence (freeze instead of cut).
    
    Args:
        mesh: The mesh data
        copy_obj: The duplicated object
        
    Returns:
        set: A set of edge keys (tuples of vertex indices) that are marked as frozen
    """
    print_subheader("PREPROCESSING USER-MARKED EDGES")
    
    # Create a BMesh to access edge data
    import bmesh
    
    # Make sure we're in object mode
    if bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    
    # Create BMesh from the mesh
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.edges.ensure_lookup_table()
    
    # Try to get the bevel weight layer
    try:
        bevel_layer = bm.edges.layers.float.get('bevel_weight_edge')
        if bevel_layer is None:
            bevel_layer = bm.edges.layers.float.get('bevel_weight')
            if bevel_layer is None:
                bevel_layer = bm.edges.layers.float.new('bevel_weight_edge')
                print("  Created new bevel weight layer 'bevel_weight_edge'")
    except Exception as e:
        print(f"  Warning: Could not access bevel weight layer: {e}")
        print("  No frozen edges will be processed")
        bm.free()
        return set()
    
    # Identify edges marked as seams (to cut) and bevel weight > 0 (to freeze)
    seam_edges = []
    frozen_edge_keys = set()
    
    for edge in bm.edges:
        try:
            # Check for bevel weight first (takes precedence)
            bevel_value = edge[bevel_layer]
            
            # Get the edge key (vertex indices)
            v0, v1 = edge.verts
            edge_key = (v0.index, v1.index) if v0.index < v1.index else (v1.index, v0.index)
            
            if bevel_value > 0:
                # Mark as frozen edge
                frozen_edge_keys.add(edge_key)
                print(f"  Edge {edge_key} marked as FROZEN with bevel weight {bevel_value}")
            elif edge.seam:
                # Mark for cutting
                seam_edges.append(edge)
                print(f"  Edge {edge_key} marked for CUTTING (seam)")
        except Exception as e:
            print(f"  Warning: Could not access edge data: {e}")
    
    print(f"Found {len(seam_edges)} edges to cut and {len(frozen_edge_keys)} edges to freeze")
    
    if seam_edges:
        # Split the seam edges
        try:
            split_result = bmesh.ops.split_edges(bm, edges=seam_edges)
            
            # Update the mesh
            bm.to_mesh(mesh)
            mesh.update()
            
            print(f"Split {len(seam_edges)} seam edges")
        except Exception as e:
            print(f"  Error during edge splitting: {e}")
    else:
        print("No seam edges found to precut")
    
    # Free the BMesh
    bm.free()
    
    # Ensure we're back in Object mode
    if bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    
    return frozen_edge_keys

def create_level_based_shape_keys(obj, mesh, paths, origin_face_idx, unfold_method):
    """
    Create shape keys based on levels across all paths.
    Simplified to ensure proper accumulation of transformations between levels.
    """
    print_subheader(f"CREATING LEVEL-BASED SHAPE KEYS ({unfold_method})")
    
    # Find the maximum level across all paths
    max_level = 0
    for path in paths:
        max_level = max(max_level, path["max_level"])
    
    print(f"Maximum level across all paths: {max_level}")
    
    if max_level == 0:
        print("No levels to unfold, skipping shape key creation")
        return
    
    # Define fractions for shape keys
    fractions = [0.25, 0.50, 0.75, 1.00]
    
    # Ensure all existing shape keys are at value 0
    reset_shape_key_values(obj)
    
    # Store the original basis shape key
    basis_key = obj.data.shape_keys.key_blocks["Basis"]
    
    # Determine the order of levels to process
    if unfold_method == 'TRUNK_FIRST':
        # Process from level 1 to max_level (trunk first)
        level_order = list(range(1, max_level + 1))
    else:  # BRANCHES_FIRST
        # Process from max_level down to level 1 (branches first)
        level_order = list(range(max_level, 0, -1))
    
    print(f"Processing levels in order: {level_order}")
    
    # Create a vertex position tracker dictionary to maintain the current state
    # This will be updated as we progress through ALL levels and fractions
    vertex_positions = {}
    for v_idx in range(len(mesh.vertices)):
        vertex_positions[v_idx] = basis_key.data[v_idx].co.copy()
    
    # Track the last shape key created for each level
    last_level_shape_key = {}
    
    # Track the previous level processed to set up correct relative keys
    previous_level_processed = None
    
    # Process each level in the determined order
    for level_idx, current_level in enumerate(level_order):
        print(f"\nProcessing level {current_level} ({level_idx+1}/{len(level_order)}):")
        
        # Create shape keys for this level at different fractions
        first_key_for_level = None
        previous_key_for_level = None
        previous_fraction = 0.0  # Track the previous fraction for incremental rotations
        
        # For each fraction (0.25, 0.50, 0.75, 1.00), create a shape key
        for frac_idx, frac in enumerate(fractions):
            # Create a unique name for this level's shape key
            sk_name = f"Level-{unfold_method}-{current_level:02d}-{int(frac*100)}%"
            print(f"  Creating shape key: {sk_name}")
            
            # Create new shape key from mix (to build on previous keys)
            shape_key = obj.shape_key_add(name=sk_name, from_mix=True)
            
            # Store reference to first key for this level
            if first_key_for_level is None:
                first_key_for_level = shape_key
                
                # IMPORTANT FIX: For the first shape key of each level (except the first level processed),
                # set it relative to the 100% shape key of the previous level
                if previous_level_processed is not None and previous_level_processed in last_level_shape_key:
                    previous_level_last_key = last_level_shape_key[previous_level_processed]
                    shape_key.relative_key = previous_level_last_key
                    print(f"  FIXED: Set first key of level {current_level} ({shape_key.name}) relative to last key of level {previous_level_processed} ({previous_level_last_key.name})")
            
            # Start with the CURRENT accumulated vertex positions
            for v_idx in range(len(mesh.vertices)):
                shape_key.data[v_idx].co = vertex_positions[v_idx].copy()
            
            # Calculate the incremental fraction to apply (only the difference from previous fraction)
            incremental_fraction = frac - previous_fraction
            print(f"  Applying incremental rotation: {int(incremental_fraction*100)}% (from {int(previous_fraction*100)}% to {int(frac*100)}%)")
            
            # Process each path
            for path_idx, path in enumerate(paths):
                # Organize faces by level for this path
                faces_by_level = {}
                for face_idx, face_data in path["faces"].items():
                    face_level = face_data["level"]
                    if face_level not in faces_by_level:
                        faces_by_level[face_level] = []
                    faces_by_level[face_level].append(face_idx)
                
                # Skip if this level doesn't exist in this path
                if current_level not in faces_by_level:
                    continue
                
                print(f"    Path {path_idx}: Processing level {current_level} faces")
                
                # Process all faces at the current level for this path
                for face_idx in faces_by_level[current_level]:
                    face_data = path["faces"][face_idx]
                    parent_idx = face_data["parent"]
                    
                    if parent_idx is None:
                        print(f"      ERROR: Face {face_idx}: No parent found, skipping")
                        continue
                    
                    # Get stored connecting edge
                    connecting_edge = face_data["connecting_edge"]
                    if connecting_edge is None:
                        print(f"      ERROR: Face {face_idx}: No connecting edge found, skipping")
                        continue
                    
                    # Get angle to use
                    initial_angle = face_data["initial_unfold_angle"]
                    if initial_angle is None:
                        print(f"      ERROR: Face {face_idx} has no initial unfold angle, skipping rotation")
                        continue
                    
                    # Use only the incremental angle based on the difference between fractions
                    use_angle = initial_angle * incremental_fraction
                    
                    # CRITICAL: Use CURRENT vertex positions for hinge points
                    vA_idx, vB_idx = connecting_edge
                    vA_co = vertex_positions[vA_idx]
                    vB_co = vertex_positions[vB_idx]
                    
                    # Calculate the rotation axis vector
                    axis_vec = (vB_co - vA_co).normalized()
                    
                    print(f"      Rotating face {face_idx} (child of {parent_idx}) around hinge {connecting_edge}")
                    print(f"      Incremental angle: {math.degrees(use_angle):.2f}° ({int(incremental_fraction*100)}% of {math.degrees(initial_angle):.2f}°)")
                    
                    # Get all vertices in this face's subtree (this face and all its descendants)
                    subtree_verts = collect_subtree_vertices(mesh, path, face_idx)
                    print(f"      Rotating {len(subtree_verts)} vertices in subtree of face {face_idx}")
                    
                    # Create rotation matrix
                    rot_mat = mathutils.Matrix.Rotation(use_angle, 4, axis_vec)
                    
                    # Apply rotation to all vertices in the subtree
                    for v_idx in subtree_verts:
                        # Start with CURRENT position
                        current_pos = vertex_positions[v_idx]
                        
                        # Make position local to hinge point A
                        local_pos = current_pos - vA_co
                        
                        # Apply rotation
                        rotated_pos = rot_mat @ local_pos
                        
                        # Convert back to global coordinates
                        final_pos = rotated_pos + vA_co
                        
                        # Update both the tracking dictionary and shape key
                        vertex_positions[v_idx] = final_pos
                        shape_key.data[v_idx].co = final_pos
            
            # Set relative key relationship within this level (for all except the first key of the level)
            if previous_key_for_level and shape_key != first_key_for_level:
                shape_key.relative_key = previous_key_for_level
                print(f"    Set {sk_name} relative to {previous_key_for_level.name}")
            
            # Store for next iteration
            previous_key_for_level = shape_key
            previous_fraction = frac  # Update the previous fraction for the next iteration
            
            # If this is the last fraction (100%), store this shape key as the final state for this level
            if frac == fractions[-1]:
                last_level_shape_key[current_level] = shape_key
                print(f"    Stored final state for level {current_level}")
        
        # CRITICAL: After completing a level, ensure we're using the fully rotated state (100%)
        # for the next level by explicitly setting all vertex positions to the final state
        print(f"  Ensuring vertex positions are at 100% rotation for level {current_level} before proceeding to next level")
        print(f"  IMPORTANT: vertex_positions dictionary is maintained between levels - Level {current_level} is now fully rotated")
        # The vertex_positions dictionary already contains the fully rotated positions from the last shape key (100%)
        # This is critical for the next level to start from the correct positions
        # DO NOT reset vertex_positions between levels - this ensures proper accumulation of transformations
        
        # Update the previous level processed
        previous_level_processed = current_level
    
    # Set up driver for automatic sequence
    setup_level_sequential_drivers(obj, unfold_method, fractions)
    
    # Reset all shape keys to 0 before returning
    reset_shape_key_values(obj)
    print(f"Completed all level-based shape keys for {unfold_method}")

def setup_level_sequential_drivers(obj, unfold_method, fractions):
    """
    Set up drivers for sequential activation of shape keys based on levels.
    
    This function creates a control property for each level and a master control
    that allows for sequential activation of all levels in order.
    
    Args:
        obj: The Blender object with shape keys
        unfold_method: The unfolding method (TRUNK_FIRST or BRANCHES_FIRST)
        fractions: List of fractions used for shape keys (e.g., [0.25, 0.5, 0.75, 1.0])
    """
    print(f"Setting up level sequential drivers for {unfold_method}")
    
    # Get shape keys
    if not obj.data.shape_keys:
        print("No shape keys found on object")
        return
        
    shape_keys = obj.data.shape_keys.key_blocks
    
    # Organize shape keys by level
    level_keys = {}
    max_level = 0
    
    # Expected format: "Level-UNFOLD_METHOD-XX-Y%"
    for key in shape_keys:
        if key.name == "Basis":
            continue
            
        parts = key.name.split('-')
        if len(parts) >= 3 and parts[0] == "Level":
            try:
                # Extract level number from the format "Level-METHOD-XX-Y%"
                level = int(parts[2])
                max_level = max(max_level, level)
                
                if level not in level_keys:
                    level_keys[level] = []
                    
                level_keys[level].append(key)
                print(f"Added shape key {key.name} to level {level}")
            except (ValueError, IndexError):
                print(f"Could not parse level from {key.name}")
    
    if not level_keys:
        print("No valid level-based shape keys found")
        return
        
    print(f"Found {len(level_keys)} levels with max level {max_level}")
    
    # Ensure master control property exists
    master_control_name = f"00_{unfold_method}_Master_Control"
    if master_control_name not in obj:
        obj[master_control_name] = 0.0
        obj.id_properties_ui(master_control_name).update(min=0.0, max=1.0)
        print(f"Added master control property {master_control_name} to object")
    
    # Create a control property for each level
    for level in level_keys:
        control_name = f"{unfold_method}_Level_{level:02d}_Control"
        
        if control_name not in obj:
            obj[control_name] = 0.0
            obj.id_properties_ui(control_name).update(min=0.0, max=1.0)
            print(f"Added level control property {control_name} to object")
    
    # Determine the order of levels based on the unfolding method
    level_order = sorted(level_keys.keys())
    if unfold_method == 'BRANCHES_FIRST':
        level_order.reverse()
    
    # Calculate the segment size for each level in the master control
    num_levels = len(level_keys)
    level_segment_size = 1.0 / num_levels if num_levels > 0 else 1.0
    
    # Helper function to extract percentage from shape key name
    def extract_percentage(key_name):
        parts = key_name.split('-')
        if len(parts) >= 4 and '%' in parts[-1]:
            try:
                percent_str = parts[-1].replace('%', '')
                return float(percent_str) / 100.0
            except ValueError:
                return 0.0
        return 0.0
    
    # Set up drivers for each level's shape keys
    for level_idx, level in enumerate(level_order):
        if level not in level_keys:
            continue
            
        # Sort keys by percentage
        keys = sorted(level_keys[level], key=lambda k: extract_percentage(k.name))
        
        # Get all percentages for this level
        level_fractions = []
        for key in keys:
            percent = extract_percentage(key.name)
            if percent > 0:
                level_fractions.append(percent)
        
        # Sort fractions
        level_fractions.sort()
        
        # Calculate the master control range for this level
        master_lower = level_idx * level_segment_size
        master_upper = (level_idx + 1) * level_segment_size
        
        # Set up drivers for each shape key in this level
        for key in keys:
            # Extract percentage from name
            percent = extract_percentage(key.name)
            if percent <= 0:
                continue
                
            # Create new driver
            fcurve = key.driver_add("value")
            driver = fcurve.driver
            driver.type = 'SCRIPTED'
            
            # Add variable for the level control property
            var1 = driver.variables.new()
            var1.name = "levelcontrol"
            var1.type = 'SINGLE_PROP'
            var1.targets[0].id = obj
            var1.targets[0].data_path = f'["{unfold_method}_Level_{level:02d}_Control"]'
            
            # Add variable for the master control property
            var2 = driver.variables.new()
            var2.name = "mastercontrol"
            var2.type = 'SINGLE_PROP'
            var2.targets[0].id = obj
            var2.targets[0].data_path = f'["{master_control_name}"]'
            
            # Find the lower bound based on the level fractions
            level_lower_bound = 0.0
            for i, frac in enumerate(level_fractions):
                if frac == percent:
                    level_lower_bound = 0.0 if i == 0 else level_fractions[i-1]
                    break
            
            level_upper_bound = percent
            level_range_size = level_upper_bound - level_lower_bound
            
            # Calculate the range for the master control
            master_fraction_lower = master_lower + (level_lower_bound * level_segment_size)
            master_fraction_upper = master_lower + (level_upper_bound * level_segment_size)
            master_range_size = master_fraction_upper - master_fraction_lower
            
            # Create a simpler driver expression that works the same way
            if level_range_size > 0 and master_range_size > 0:
                driver_expr = f"max(min(1, (levelcontrol - {level_lower_bound:.4f}) / {level_range_size:.4f} if levelcontrol >= {level_lower_bound:.4f} else 0), min(1, (mastercontrol - {master_fraction_lower:.4f}) / {master_range_size:.4f} if mastercontrol >= {master_fraction_lower:.4f} else 0))"
                driver.expression = driver_expr
                print(f"Set driver for {key.name}: active from {level_lower_bound:.4f} to {level_upper_bound:.4f}")
    
    print(f"Completed setting up level sequential drivers for {unfold_method}")

def preserve_origin_face_data(mesh, origin_face_idx):
    """
    Store the origin face data (vertex positions) before edge splitting.
    Uses only coordinate locations to ensure we can find the face after topology changes.
    Returns a dictionary with the origin face data.
    """
    print_subheader("PRESERVING ORIGIN FACE DATA")
    
    origin_face = mesh.polygons[origin_face_idx]
    
    # Store the actual coordinate locations of the vertices
    vertex_positions = []
    for v_idx in origin_face.vertices:
        vertex_positions.append(mesh.vertices[v_idx].co.copy())
    
    # Calculate the center of the face
    center = Vector((0, 0, 0))
    for pos in vertex_positions:
        center += pos
    center /= len(vertex_positions)
    
    # Store the data
    origin_data = {
        "original_index": origin_face_idx,  # Just for reference
        "vertex_count": len(origin_face.vertices),
        "vertex_positions": vertex_positions,
        "center": center,
        "normal": origin_face.normal.copy(),
        "area": origin_face.area
    }
    
    print(f"  Preserved data for origin face {origin_face_idx}")
    print(f"  Vertex count: {origin_data['vertex_count']}")
    print(f"  Center: {origin_data['center']}")
    print(f"  Normal: {origin_data['normal']}")
    print(f"  Area: {origin_data['area']}")
    
    return origin_data

def restore_origin_face(copy_obj, mesh, origin_data):
    """
    Find the face that matches the stored origin face data after edge splitting.
    Simply looks for a face with the same vertex positions as the original.
    Returns the index of the new origin face.
    """
    print_subheader("RESTORING ORIGIN FACE")
    
    # Get the original vertex positions
    orig_positions = origin_data["vertex_positions"]
    orig_vertex_count = origin_data["vertex_count"]
    
    print(f"  Looking for face with {orig_vertex_count} vertices at the same positions")
    
    # First, filter by vertex count
    for f in mesh.polygons:
        if len(f.vertices) != orig_vertex_count:
            continue
        
        # Get the actual vertex positions for this face
        face_positions = [mesh.vertices[v_idx].co for v_idx in f.vertices]
        
        # Check if all vertices match (within a small tolerance)
        all_verts_match = True
        tolerance = 0.0001  # Small tolerance for floating point comparison
        
        # For each vertex in the original face, find a matching vertex in the current face
        for orig_pos in orig_positions:
            found_match = False
            for pos in face_positions:
                if (pos - orig_pos).length < tolerance:
                    found_match = True
                    break
            
            if not found_match:
                all_verts_match = False
                break
        
        # If all vertices match, we found our face
        if all_verts_match:
            print(f"  Found exact match: Face {f.index}")
            
            # Update the origin vertex group if it exists
            for vg in copy_obj.vertex_groups:
                if vg.name.lower() == 'origin':
                    # Clear the vertex group
                    for v_idx in range(len(mesh.vertices)):
                        try:
                            vg.remove([v_idx])
                        except:
                            pass
                    
                    # Add the vertices of the new origin face
                    for v_idx in f.vertices:
                        vg.add([v_idx], 1.0, 'REPLACE')
                    
                    print(f"  Updated 'origin' vertex group with new origin face vertices")
                    break
            
            return f.index
    
    # If we get here, we couldn't find an exact match
    print("  WARNING: Could not find an exact match for the origin face!")
    
    # Fall back to the center-based approach
    orig_center = origin_data["center"]
    closest_face_idx = None
    closest_dist = float('inf')
    
    for f in mesh.polygons:
        face_center = Vector((0, 0, 0))
        for v_idx in f.vertices:
            face_center += mesh.vertices[v_idx].co
        face_center /= len(f.vertices)
        
        dist = (face_center - orig_center).length
        if dist < closest_dist:
            closest_dist = dist
            closest_face_idx = f.index
    
    if closest_face_idx is not None:
        print(f"  Falling back to closest face by center: {closest_face_idx} (distance: {closest_dist:.6f})")
        return closest_face_idx
    else:
        # Last resort: use the first face
        print(f"  CRITICAL ERROR: Could not find any suitable face. Using face 0 as fallback.")
        return 0

def unfold_papercraft(unfold_method='BY_PATH', cleanup_option='NONE'):
    """Main function for the papercraft unfolding script."""
    print_header("STARTING UNFOLD PAPERCRAFT SCRIPT")
    print(f"Using unfolding method: {unfold_method}")
    print(f"Using cleanup option: {cleanup_option}")

    # Always start in Object Mode to ensure consistent behavior
    if bpy.context.object and bpy.context.object.mode != 'OBJECT':
        print("Switching to Object Mode before starting")
        bpy.ops.object.mode_set(mode='OBJECT')

    # 1) Get active object
    obj = bpy.context.active_object
    if not obj or obj.type != 'MESH':
        print("Please select a valid mesh object.")
        return

    original_name = obj.name
    print(f"Processing object: {original_name}")

    # 2) Duplicate the object
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.duplicate()
    copy_obj = bpy.context.active_object
    copy_obj.name = f"{original_name}-flat"
    mesh = copy_obj.data
    mesh.update()
    print(f"Created duplicate object: {copy_obj.name}")
    print(f"Mesh stats: {len(mesh.vertices)} vertices, {len(mesh.edges)} edges, {len(mesh.polygons)} faces")
    
    # 2.25) Check if the original object has an 'origin' vertex group and transfer it to the copy
    origin_group_idx = None
    for vg in obj.vertex_groups:
        if vg.name.lower() == 'origin':
            print(f"Found 'origin' vertex group in original object: {vg.name}")
            # Check if the copy already has this group (it should, but let's be safe)
            copy_origin_group = None
            for copy_vg in copy_obj.vertex_groups:
                if copy_vg.name.lower() == 'origin':
                    copy_origin_group = copy_vg
                    break
            
            if not copy_origin_group:
                print("  Creating 'origin' vertex group in copy")
                copy_origin_group = copy_obj.vertex_groups.new(name='origin')
            
            # Transfer weights from original to copy
            for v_idx in range(len(mesh.vertices)):
                try:
                    weight = vg.weight(v_idx)
                    copy_origin_group.add([v_idx], weight, 'REPLACE')
                except RuntimeError:
                    # This vertex is not in the group
                    pass
            
            print(f"  Transferred 'origin' vertex group to copy")
            break

    # 2.5) Identify and split seam edges (precuts) and identify frozen edges (bevel weight > 0)
    frozen_edges = identify_and_split_seam_edges(mesh, copy_obj)
    print(f"Identified {len(frozen_edges)} edges that will maintain their folded angle")
    
    # Print updated mesh stats after precuts
    mesh.update()
    print(f"Mesh stats after precuts: {len(mesh.vertices)} vertices, {len(mesh.edges)} edges, {len(mesh.polygons)} faces")
    
    # 3) Pick the origin face - pass the copy object to check for vertex groups
    # Changed from original object to copy object since we've transferred the vertex group
    origin_face_idx = pick_origin_face(copy_obj, mesh)
    origin_face = mesh.polygons[origin_face_idx]
    print(f"Origin face has {len(origin_face.vertices)} vertices")
    
    # 3.5) Preserve the origin face data before any edge splitting
    origin_data = preserve_origin_face_data(mesh, origin_face_idx)

    # 4) Build mesh connectivity
    face_adjacency, edge_to_faces, face_to_edges = build_mesh_connectivity(mesh)

    # 4.5) Identify precut edges in the updated mesh topology
    # These are edges that connect to only one face (boundary edges)
    precut_edges = set()
    for edge_key, faces in edge_to_faces.items():
        if len(faces) == 1:
            precut_edges.add(edge_key)
    
    print(f"Identified {len(precut_edges)} boundary edges from precuts")

    # 5) Initialize paths - one for each edge of the origin face
    paths = initialize_paths(mesh, origin_face_idx, face_adjacency, face_to_edges)

    print_subheader("DEBUG - PATHS AFTER INITIALIZATION")
    for path_check in paths:
        print(f"  Path ID: {path_check['id']}, Keys: {path_check.keys()}, Origin Face Index: {path_check.get('origin_face_idx', 'Missing')}")

    # 6) Grow paths until all faces are covered or no more growth is possible
    print_subheader("GROWING PATHS")
    
    # Track which faces have been assigned to paths
    assigned_faces = set()
    # 6) Set up available pools
    available_faces = set(f.index for f in mesh.polygons)
    available_faces.remove(origin_face_idx)  # Origin face is already claimed

    available_edges = set()
    for edge_key in edge_to_faces.keys():
        # Only add edges that connect to exactly 2 faces (non-boundary edges)
        if len(edge_to_faces[edge_key]) == 2:
            available_edges.add(edge_key)
    
    print(f"Available edges after filtering boundary edges: {len(available_edges)}")

    split_edges = set()
    split_edges.update(precut_edges)  # Add precut edges to split edges
    outer_edges = set()

    # 7) PHASE 1: Forward Growth - Perform turn-based growth prioritizing forward progress
    print_subheader("PHASE 1: FORWARD GROWTH")

    forward_growth_complete = False
    max_rounds = len(mesh.polygons) * 3  # Safety limit for forward growth phase
    round_num = 0

    while not forward_growth_complete and round_num < max_rounds:
        round_num += 1
        print(f"\nForward Growth Round {round_num}:")

        paths_grown = 0

        for i, path in enumerate(paths):
            print(f"\n  Path {i} turn:")
            path_grown, faces_at_next_level, available_faces, available_edges, split_edges, outer_edges = grow_path_one_step(
                path, available_faces, available_edges, face_adjacency, edge_to_faces, face_to_edges,
                mesh, copy_obj, split_edges, outer_edges, frozen_edges
            )
            
            if path_grown:
                paths_grown += 1

        print(f"\nForward Growth Round {round_num} results: {paths_grown} paths grown")
        print(f"  Remaining available faces: {len(available_faces)}")
        print(f"  Remaining available edges: {len(available_edges)}")

        if paths_grown == 0:
            forward_growth_complete = True

    print_subheader("FORWARD GROWTH COMPLETED")
    print(f"Completed after {round_num} rounds")
    print(f"Faces remaining after forward growth: {len(available_faces)}")
    
    # 8) PHASE 2: Cleanup - Revisit all faces in each path to connect any stranded faces
    if available_faces:
        print_subheader("PHASE 2: CLEANUP PASSES")
        print(f"Starting cleanup to connect {len(available_faces)} remaining faces")
        
        cleanup_complete = False
        max_cleanup_rounds = len(mesh.polygons) * 3  # Safety limit for cleanup phase
        cleanup_round = 0
        
        while not cleanup_complete and cleanup_round < max_cleanup_rounds and available_faces:
            cleanup_round += 1
            print(f"\nCleanup Round {cleanup_round}:")
            
            faces_connected = 0
            
            # Process each path
            for path_idx, path in enumerate(paths):
                print(f"\n  Cleanup for Path {path_idx}:")
                
                # Process each level from 0 (origin) upward
                max_level = path["max_level"]
                for level in range(max_level + 1):
                    if level not in path["levels"]:
                        continue
                        
                    print(f"    Checking level {level} faces:")
                    level_faces = list(path["levels"][level])
                    
                    # Check each face at this level for potential connections
                    for face_idx in level_faces:
                        # Skip if no available faces left
                        if not available_faces:
                            break
                            
                        print(f"      Checking face {face_idx} for potential connections")
                        
                        # Get all edges of this face
                        face_edges = face_to_edges[face_idx]
                        
                        # Find available edges that belong to this face
                        available_face_edges = [e for e in face_edges if e in available_edges]
                        
                        if not available_face_edges:
                            print(f"        No available edges on face {face_idx}")
                            continue
                        
                        # Filter out boundary edges (edges that only connect to one face)
                        valid_edges = []
                        for edge in available_face_edges:
                            if len(edge_to_faces[edge]) == 2:
                                valid_edges.append(edge)
                            else:
                                print(f"        Edge {edge} is a boundary edge (connects to only {len(edge_to_faces[edge])} face), skipping.")
                        
                        if not valid_edges:
                            print(f"        No valid non-boundary edges on face {face_idx} to traverse.")
                            continue
                        
                        # Prioritize frozen edges during cleanup
                        frozen_face_edges = [e for e in valid_edges if e in frozen_edges]
                        if frozen_face_edges:
                            # Process frozen edges first
                            valid_edges = frozen_face_edges + [e for e in valid_edges if e not in frozen_edges]
                            
                        # Try each available edge
                        for edge in valid_edges:
                            # Find the face on the other side of this edge
                            edge_faces = edge_to_faces[edge]
                            other_face_idx = edge_faces[0] if edge_faces[1] == face_idx else edge_faces[1]
                            
                            # Skip if the other face is already in this path or not available
                            if other_face_idx in path["face_indices"] or other_face_idx not in available_faces:
                                continue
                                
                            # We found a face to connect!
                            print(f"        Connecting face {other_face_idx} to face {face_idx} via edge {edge}")
                            
                            # Update path data
                            if (level + 1) not in path["levels"]:
                                path["levels"][level + 1] = set()
                            path["levels"][level + 1].add(other_face_idx)
                            path["max_level"] = max(path["max_level"], level + 1)
                            path["face_indices"].add(other_face_idx)
                            path["faces"][other_face_idx] = {
                                "parent": face_idx,
                                "connecting_edge": edge,
                                "children": [],
                                "level": level + 1,
                                "initial_unfold_angle": None,
                                "direction_reason_initial_angle": None
                            }
                            path["faces"][face_idx]["children"].append(other_face_idx)
                            path["edges"].add(edge)
                            
                            # Update available pools
                            available_faces.remove(other_face_idx)
                            available_edges.discard(edge)
                            
                            faces_connected += 1
            
            print(f"\nCleanup Round {cleanup_round} results: {faces_connected} faces connected")
            print(f"  Remaining available faces: {len(available_faces)}")
            
            if faces_connected == 0:
                cleanup_complete = True
        
        print_subheader("CLEANUP COMPLETED")
        print(f"Completed after {cleanup_round} cleanup rounds")
        print(f"Faces remaining after cleanup: {len(available_faces)}")
        
        if available_faces:
            print("WARNING: Some faces could not be connected to any path!")
            print(f"Unconnected faces: {available_faces}")
    
    # 9) Print final path summaries
    print_subheader("PATH SUMMARIES")
    for path in paths:
        print_path_summary(path)

    # 10) COLLECTING FOLD EDGES
    print_subheader("COLLECTING FOLD EDGES")
    all_fold_edges = set()
    for path in paths:
        all_fold_edges.update(path["edges"])
    print(f"Total fold edges across all paths: {len(all_fold_edges)}")
    print(f"Fold edges: {all_fold_edges}")

    # 11) Split non-fold edges and DUPLICATE VERTS
    split_edges_and_duplicate_verts(mesh, all_fold_edges)
    
    # Store freestyle edges after splitting
    print_subheader("STORING FREESTYLE EDGES AFTER SPLITTING")
    freestyle_edge_indices = set()
    for edge in mesh.edges:
        if edge.use_freestyle_mark:
            freestyle_edge_indices.add(edge.index)
            print(f"Stored freestyle edge at index {edge.index}")
    print(f"Total freestyle edges stored: {len(freestyle_edge_indices)}")
    
    # 11.5) Restore the origin face after edge splitting
    mesh.update()
    origin_face_idx = restore_origin_face(copy_obj, mesh, origin_data)
    
    # Update the origin face index in all paths
    for path in paths:
        path["origin_face_idx"] = origin_face_idx
    
    # 12) Calculate and store unfolding angles AFTER edge splitting
    calculate_post_split_angles(mesh, paths, copy_obj, frozen_edges)

    # 13) Create Basis Shape Key ONCE, before path loop
    print_subheader("ENSURING BASIS SHAPE KEY EXISTS")
    if not copy_obj.data.shape_keys:
        copy_obj.shape_key_add(name="Basis", from_mix=False)
        print("  Created Basis shape key (once, before paths)")

    # 14) Create shape keys based on the selected unfolding method
    print_subheader(f"CREATING SHAPE KEYS USING METHOD: {unfold_method}")
    
    if unfold_method == 'BY_PATH':
        # Original method - create shape keys for each path independently
        for path in paths:
            create_shape_keys_for_path(copy_obj, mesh, path, origin_face_idx)
            # Reset Shape Key Values AFTER each path
            reset_shape_key_values(copy_obj)
    else:
        # Level-based methods (TRUNK_FIRST or BRANCHES_FIRST)
        create_level_based_shape_keys(copy_obj, mesh, paths, origin_face_idx, unfold_method)

    # Print final mesh stats
    mesh.update()
    print(f"Final mesh stats: {len(mesh.vertices)} vertices, {len(mesh.edges)} edges, {len(mesh.polygons)} faces")

    # Apply cleanup options if selected
    if cleanup_option != 'NONE':
        print_subheader(f"APPLYING POST-PROCESSING CLEANUP: {cleanup_option}")
        
        # Select the copy object and enter edit mode
        bpy.ops.object.select_all(action='DESELECT')
        copy_obj.select_set(True)
        bpy.context.view_layer.objects.active = copy_obj
        
        # Create a BMesh for editing
        import bmesh
        
        # Remove edge marks (bevel weights and seams)
        if cleanup_option in ['MARKS', 'FACES', 'ALL']:
            print("  Removing edge marks (bevel weights and seams)")
            
            # Use object mode for this operation
            bpy.ops.object.mode_set(mode='OBJECT')
            
            # Create a new BMesh
            bm = bmesh.new()
            bm.from_mesh(copy_obj.data)
            
            # Try to get the bevel weight layer
            bevel_layer = None
            
            try:
                bevel_layer = bm.edges.layers.float.get('bevel_weight_edge')
                if bevel_layer is None:
                    bevel_layer = bm.edges.layers.float.get('bevel_weight')
            except Exception as e:
                print(f"    Warning: Could not access bevel weight layer: {e}")
            
            # Clear bevel weights if layer exists
            if bevel_layer:
                for edge in bm.edges:
                    edge[bevel_layer] = 0.0
                print("    Cleared all bevel weights")
            
            # Clear seams (no special layer needed)
            seam_count = 0
            for edge in bm.edges:
                if edge.seam:
                    edge.seam = False
                    seam_count += 1
            if seam_count > 0:
                print(f"    Cleared {seam_count} seam marks")
            
            # Clear all edge selections
            for edge in bm.edges:
                edge.select = False
            
            # Update the mesh
            bm.to_mesh(copy_obj.data)
            bm.free()
            copy_obj.data.update()
            
            # Switch to edit mode and back to ensure selection state is updated
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='DESELECT')
            bpy.ops.object.mode_set(mode='OBJECT')
        
        # For face and edge deletion, use Blender's built-in operators
        if cleanup_option in ['FACES', 'ALL_BUT_FROZEN', 'ALL']:
            print("  Removing all faces")
            
            # Switch to edit mode
            bpy.ops.object.mode_set(mode='EDIT')
            
            # Set selection mode to faces
            bpy.ops.mesh.select_mode(type='FACE')
            
            # Select all faces
            bpy.ops.mesh.select_all(action='SELECT')
            
            # Delete faces only, keeping edges and vertices
            bpy.ops.mesh.delete(type='ONLY_FACE')
            
            print("    Removed faces, keeping all edges and vertices")
            
            # Update the mesh
            bpy.ops.object.mode_set(mode='OBJECT')
            mesh.update()
            
            # Print stats after face removal
            print(f"    After face removal: {len(mesh.vertices)} vertices, {len(mesh.edges)} edges, {len(mesh.polygons)} faces")
        
        # Remove fold edges except frozen ones
        if cleanup_option == 'ALL_BUT_FROZEN':
            print("  Removing fold edges (except frozen and freestyle ones)")
            
            # Get all fold edges from paths
            fold_edges = []
            for path in paths:
                for face_idx, face_data in path["faces"].items():
                    if "connecting_edge" in face_data and face_data["connecting_edge"]:
                        fold_edges.append(face_data["connecting_edge"])
            
            print(f"    Identified {len(fold_edges)} fold edges to check")
            
            if fold_edges:
                # First, identify which edges are frozen (have bevel weight > 0)
                frozen_edge_indices = set()
                freestyle_edge_indices = set()
                
                # Create a BMesh to access edge properties
                bpy.ops.object.mode_set(mode='OBJECT')
                bm = bmesh.new()
                bm.from_mesh(mesh)
                
                # Try to get the bevel weight layer
                bevel_layer = None
                try:
                    bevel_layer = bm.edges.layers.float.get('bevel_weight_edge')
                    if bevel_layer is None:
                        bevel_layer = bm.edges.layers.float.get('bevel_weight')
                except Exception as e:
                    print(f"    Warning: Could not access bevel weight layer: {e}")
                
                # Create a map from vertex pairs to edge indices
                edge_map = {}
                for edge in mesh.edges:
                    v1, v2 = edge.vertices
                    edge_key1 = (v1, v2)
                    edge_key2 = (v2, v1)
                    edge_map[edge_key1] = edge.index
                    edge_map[edge_key2] = edge.index
                    
                    # Check for freestyle edges using the proper property
                    if edge.use_freestyle_mark:
                        freestyle_edge_indices.add(edge.index)
                
                # If we have a bevel layer, identify frozen edges
                if bevel_layer:
                    for edge in bm.edges:
                        if edge[bevel_layer] > 0:
                            v0, v1 = edge.verts
                            edge_key1 = (v0.index, v1.index)
                            edge_key2 = (v1.index, v0.index)
                            
                            if edge_key1 in edge_map:
                                frozen_edge_indices.add(edge_map[edge_key1])
                            elif edge_key2 in edge_map:
                                frozen_edge_indices.add(edge_map[edge_key2])
                
                bm.free()
                
                print(f"    Found {len(frozen_edge_indices)} frozen edges to preserve")
                print(f"    Found {len(freestyle_edge_indices)} freestyle edges to preserve")
                
                # Switch to edit mode
                bpy.ops.object.mode_set(mode='EDIT')
                
                # Deselect all
                bpy.ops.mesh.select_all(action='DESELECT')
                
                # Set selection mode to edges
                bpy.ops.mesh.select_mode(type='EDGE')
                
                # Switch to object mode to select specific edges
                bpy.ops.object.mode_set(mode='OBJECT')
                
                # Select fold edges that are NOT frozen and NOT freestyle
                edge_count = 0
                for edge in mesh.edges:
                    if edge.index in frozen_edge_indices or edge.index in freestyle_edge_indices:
                        continue  # Skip frozen and freestyle edges
                        
                    v1, v2 = edge.vertices
                    
                    # Check if this is a fold edge
                    is_fold_edge = False
                    for fold_edge in fold_edges:
                        if (fold_edge[0] == v1 and fold_edge[1] == v2) or (fold_edge[0] == v2 and fold_edge[1] == v1):
                            is_fold_edge = True
                            break
                    
                    if is_fold_edge:
                        edge.select = True
                        edge_count += 1
                
                print(f"    Selected {edge_count} non-frozen, non-freestyle fold edges for removal")
                
                # Switch back to edit mode
                bpy.ops.object.mode_set(mode='EDIT')
                
                # Delete selected edges only, keeping vertices
                if edge_count > 0:
                    bpy.ops.mesh.delete(type='EDGE')
                    print(f"    Removed {edge_count} non-frozen, non-freestyle fold edges, keeping all vertices")
                else:
                    print("    No non-frozen, non-freestyle fold edges found to remove")
                
                # Update the mesh
                bpy.ops.object.mode_set(mode='OBJECT')
                mesh.update()
        
        # Remove all fold edges (except freestyle ones)
        if cleanup_option == 'ALL':
            print("  Removing all fold edges (except freestyle edges)")
            
            # Get all fold edges from paths
            fold_edges = []
            for path in paths:
                for face_idx, face_data in path["faces"].items():
                    if "connecting_edge" in face_data and face_data["connecting_edge"]:
                        fold_edges.append(face_data["connecting_edge"])
            
            print(f"    Identified {len(fold_edges)} fold edges to check")
            
            if fold_edges:
                # First, identify freestyle edges
                freestyle_edge_indices = set()
                
                # Check for freestyle edges using the proper property
                for edge in mesh.edges:
                    if edge.use_freestyle_mark:
                        freestyle_edge_indices.add(edge.index)
                
                print(f"    Found {len(freestyle_edge_indices)} freestyle edges to preserve")
                
                # Switch to edit mode
                bpy.ops.object.mode_set(mode='EDIT')
                
                # Deselect all
                bpy.ops.mesh.select_all(action='DESELECT')
                
                # Set selection mode to edges
                bpy.ops.mesh.select_mode(type='EDGE')
                
                # Switch to object mode to select specific edges
                bpy.ops.object.mode_set(mode='OBJECT')
                
                # Select fold edges that are NOT freestyle
                edge_count = 0
                for edge in mesh.edges:
                    if edge.index in freestyle_edge_indices:
                        continue  # Skip freestyle edges
                        
                    v1, v2 = edge.vertices
                    
                    # Check if this is a fold edge
                    is_fold_edge = False
                    for fold_edge in fold_edges:
                        if (fold_edge[0] == v1 and fold_edge[1] == v2) or (fold_edge[0] == v2 and fold_edge[1] == v1):
                            is_fold_edge = True
                            break
                    
                    if is_fold_edge:
                        edge.select = True
                        edge_count += 1
                
                print(f"    Selected {edge_count} non-freestyle fold edges for removal")
                
                # Switch back to edit mode
                bpy.ops.object.mode_set(mode='EDIT')
                
                # Delete selected edges only, keeping vertices
                if edge_count > 0:
                    bpy.ops.mesh.delete(type='EDGE')
                    print(f"    Removed {edge_count} non-freestyle fold edges, keeping all vertices")
                else:
                    print("    No non-freestyle fold edges found to remove")
                
                # Update the mesh
                bpy.ops.object.mode_set(mode='OBJECT')
                mesh.update()
        
        # Update mesh data after cleanup
        mesh.update()
        print(f"Post-cleanup mesh stats: {len(mesh.vertices)} vertices, {len(mesh.edges)} edges, {len(mesh.polygons)} faces")

    # Ensure we return to Object mode before finishing - FORCE it regardless of current state
    print("Ensuring Object Mode is set before finishing")
    bpy.ops.object.mode_set(mode='OBJECT')

    print_header(f"UNFOLDING COMPLETED: {copy_obj.name}")

    return copy_obj

class OBJECT_OT_unfold_papercraft_multi(bpy.types.Operator):
    """Unfold the selected mesh into a papercraft pattern with multiple paths."""
    bl_idname = "object.unfold_papercraft_multi"
    bl_label = "Unfold Papercraft (Multi-Paths)"
    bl_options = {'REGISTER', 'UNDO'}

    unfold_method: bpy.props.EnumProperty(
        name="Unfolding Method",
        description="Method to use for unfolding the mesh",
        items=[
            ('BY_PATH', "By Path", "Unfold each path independently (original method)"),
            ('TRUNK_FIRST', "Trunk First", "Unfold from trunk (origin) outward to branches"),
            ('BRANCHES_FIRST', "Branches First", "Unfold from outer branches inward to trunk")
        ],
        default='BY_PATH'
    )
    
    cleanup_option: bpy.props.EnumProperty(
        name="Cleanup",
        description="Post-processing cleanup options",
        items=[
            ('NONE', "None", "No post-processing"),
            ('MARKS', "Marks", "Remove edge marks"),
            ('FACES', "Faces", "Remove all faces (leaving wireframe)"),
            ('ALL_BUT_FROZEN', "All but Frozen", "Remove all fold edges except frozen ones"),
            ('ALL', "All", "Remove unfolded edges")
        ],
        default='NONE'
    )

    def execute(self, context):
        # Check if we're in object mode
        if context.mode != 'OBJECT':
            self.report({'ERROR'}, "Must be in Object Mode")
            return {'CANCELLED'}
            
        # Check if exactly one object is selected
        selected_objects = context.selected_objects
        if len(selected_objects) == 0:
            self.report({'ERROR'}, "No object selected")
            return {'CANCELLED'}
        elif len(selected_objects) > 1:
            self.report({'ERROR'}, "Too many objects selected. Select only one mesh object")
            return {'CANCELLED'}
            
        # Check if the selected object is a mesh
        obj = selected_objects[0]
        if obj.type != 'MESH':
            self.report({'ERROR'}, "Selected object must be a mesh")
            return {'CANCELLED'}
        
        # Get the method from the window manager property
        unfold_method = context.window_manager.papercraft_unfold_method
        
        # Get the cleanup option from the window manager property
        cleanup_option = context.window_manager.papercraft_cleanup_option
        
        # Update the operator properties to match (for redo panel)
        self.unfold_method = unfold_method
        self.cleanup_option = cleanup_option
        
        # Call the main function with the selected method and cleanup option
        try:
            unfold_papercraft(unfold_method=unfold_method, cleanup_option=cleanup_option)
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Error during unfolding: {str(e)}")
            import traceback
            traceback.print_exc()
            return {'CANCELLED'}

class OBJECT_OT_set_origin_face(bpy.types.Operator):
    """Set the selected face as the origin face for papercraft unfolding"""
    bl_idname = "object.set_origin_face"
    bl_label = "Set Origin Face"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # This operator is only available in Edit Mode with a mesh object
        return context.object and context.object.type == 'MESH'
    
    def execute(self, context):
        obj = context.active_object
        
        # Check if we're in Edit Mode
        if obj.mode != 'EDIT':
            self.report({'ERROR'}, "You must be in Edit Mode to set the origin face")
            return {'CANCELLED'}
        
        # Get the mesh data
        bpy.ops.object.mode_set(mode='OBJECT')  # Need to switch to Object mode to access selection
        mesh = obj.data
        
        # Get selected vertices
        selected_verts = [v.index for v in mesh.vertices if v.select]
        
        if not selected_verts:
            bpy.ops.object.mode_set(mode='EDIT')  # Switch back to Edit mode
            self.report({'ERROR'}, "No vertices selected. Please select a face")
            return {'CANCELLED'}
        
        # Check if the selected vertices form exactly one face
        selected_faces = []
        for face in mesh.polygons:
            face_verts = set(face.vertices)
            if all(v in selected_verts for v in face_verts) and len(face_verts) == len(selected_verts):
                selected_faces.append(face)
        
        if len(selected_faces) != 1:
            bpy.ops.object.mode_set(mode='EDIT')  # Switch back to Edit mode
            if len(selected_faces) == 0:
                self.report({'ERROR'}, "Selected vertices do not form a complete face. Please select all vertices of a single face")
            else:
                self.report({'ERROR'}, f"Selected vertices form {len(selected_faces)} faces. Please select exactly one face")
            return {'CANCELLED'}
        
        # We have exactly one face
        origin_face = selected_faces[0]
        
        # Store the face data for debugging
        face_center = Vector((0, 0, 0))
        for v_idx in origin_face.vertices:
            face_center += mesh.vertices[v_idx].co
        face_center /= len(origin_face.vertices)
        
        # Delete any existing 'origin' vertex group
        for vg in obj.vertex_groups:
            if vg.name.lower() == 'origin':
                obj.vertex_groups.remove(vg)
                break
        
        # Create a new 'origin' vertex group
        origin_group = obj.vertex_groups.new(name='origin')
        
        # Add the selected vertices to the group
        for v_idx in selected_verts:
            origin_group.add([v_idx], 1.0, 'REPLACE')
        
        # Switch back to Edit mode
        bpy.ops.object.mode_set(mode='EDIT')
        
        self.report({'INFO'}, f"Origin face set with {len(selected_verts)} vertices at center {face_center}")
        return {'FINISHED'}

class OBJECT_OT_clear_origin_face(bpy.types.Operator):
    """Clear the origin face vertex group"""
    bl_idname = "object.clear_origin_face"
    bl_label = "Clear Origin Face"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # This operator is available as long as there's a mesh object selected
        return context.object and context.object.type == 'MESH'
    
    def execute(self, context):
        obj = context.active_object
        
        # Store current mode to restore it later
        current_mode = obj.mode
        
        # Switch to Object mode if needed
        if current_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        # Check for and remove any 'origin' vertex group
        found = False
        for vg in obj.vertex_groups:
            if vg.name.lower() == 'origin':
                obj.vertex_groups.remove(vg)
                found = True
                break
        
        # Restore original mode
        if current_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=current_mode)
        
        if found:
            self.report({'INFO'}, "Origin face vertex group removed")
        else:
            self.report({'INFO'}, "No origin face vertex group found to remove")
        
        return {'FINISHED'}

class OBJECT_OT_papercraft_help(bpy.types.Operator):
    """Display help information for the Unbender tool"""
    bl_idname = "object.papercraft_help"
    bl_label = "Unbender Help"
    bl_options = {'REGISTER', 'INTERNAL'}
    
    def execute(self, context):
        return {'FINISHED'}
    
    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=350)
    
    def draw(self, context):
        layout = self.layout
        
        # Help content
        layout.label(text="Unbender Tool", icon='INFO')
        layout.separator()
        
        # Edge Marking
        box = layout.box()
        box.label(text="Edge Marking:", icon='EDGESEL')
        col = box.column(align=True)
        col.label(text="• Precut: Mark edges to be cut before processing")
        col.label(text="• Freeze: Lock edge angle in place during unbending")
        col.label(text="• Freestyle: Mark edges for cosmetic/visual purposes")
        col.label(text="Note: Freeze takes precedence over Precut if both are set")
        
        # Unfolding Methods
        box = layout.box()
        box.label(text="Unfolding Methods:", icon='MOD_UVPROJECT')
        col = box.column(align=True)
        col.label(text="• By Path: Unfold each path independently")
        col.label(text="• Trunk First: Unfold from origin outward to branches")
        col.label(text="• Branches First: Unfold from outer branches inward to trunk")
        
        # Origin Face
        box = layout.box()
        box.label(text="Origin Face Selection:", icon='VERTEXSEL')
        col = box.column(align=True)
        col.label(text="• Select a face in Edit Mode and click 'Set' to define the origin face")
        col.label(text="• The origin face will remain flat during unfolding")
        col.label(text="• If no origin face is set, one will be automatically selected")
        
        # Cleanup Options
        box = layout.box()
        box.label(text="Cleanup Options:", icon='BRUSH_DATA')
        col = box.column(align=True)
        col.label(text="• None: Keep all geometry intact")
        col.label(text="• Marks: Remove edge markings only")
        col.label(text="• Faces: Remove all faces (leaving wireframe)")
        col.label(text="• All but Frozen: Remove all fold edges except frozen ones")
        col.label(text="• All: Remove all fold edges")
        
        # Controls
        box = layout.box()
        box.label(text="Controls:", icon='HAND')
        col = box.column(align=True)
        col.label(text="• Use shape key sliders to control unfolding")
        col.label(text="• Adjust the 'Value' slider to unfold all paths simultaneously")

class OBJECT_OT_mark_edges_precut(bpy.types.Operator):
    """Mark selected edges as precut (sets edge as seam)"""
    bl_idname = "object.mark_edges_precut"
    bl_label = "Mark Edges as Precut"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # Only available in Edit Mode with a mesh object
        return context.object and context.object.type == 'MESH' and context.mode == 'EDIT_MESH'
    
    def execute(self, context):
        obj = context.object
        bm = bmesh.from_edit_mesh(obj.data)
        
        # Count how many edges were marked
        marked_count = 0
        
        # Set seam for selected edges
        for edge in bm.edges:
            if edge.select:
                edge.seam = True
                marked_count += 1
        
        # Update the mesh
        bmesh.update_edit_mesh(obj.data)
        
        if marked_count > 0:
            self.report({'INFO'}, f"Marked {marked_count} edges as precut")
        else:
            self.report({'WARNING'}, "No edges selected")
            
        return {'FINISHED'}


class OBJECT_OT_mark_edges_freeze(bpy.types.Operator):
    """Mark selected edges as frozen (sets bevel weight to 1.0)"""
    bl_idname = "object.mark_edges_freeze"
    bl_label = "Mark Edges as Frozen"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # Only available in Edit Mode with a mesh object
        return context.object and context.object.type == 'MESH' and context.mode == 'EDIT_MESH'
    
    def execute(self, context):
        obj = context.object
        bm = bmesh.from_edit_mesh(obj.data)
        
        # Get bevel weight custom data layer 
        try:
            bevel_layer = bm.edges.layers.float.get('bevel_weight_edge')
            if bevel_layer is None:
                bevel_layer = bm.edges.layers.float.get('bevel_weight')
                if bevel_layer is None:
                    bevel_layer = bm.edges.layers.float.new('bevel_weight_edge')
                    print("Created new bevel weight layer 'bevel_weight_edge'")
        except Exception as e:
            self.report({'ERROR'}, f"Could not access bevel weight layer: {e}")
            return {'CANCELLED'}
        
        # Count how many edges were marked
        marked_count = 0
        
        # Set bevel weight for selected edges
        for edge in bm.edges:
            if edge.select:
                edge[bevel_layer] = 1.0
                marked_count += 1
        
        # Update the mesh
        bmesh.update_edit_mesh(obj.data)
        
        if marked_count > 0:
            self.report({'INFO'}, f"Marked {marked_count} edges as frozen")
        else:
            self.report({'WARNING'}, "No edges selected")
            
        return {'FINISHED'}


class OBJECT_OT_clear_edge_marks(bpy.types.Operator):
    """Clear all edge markings (seams, bevel weights, and freestyle marks)"""
    bl_idname = "object.clear_edge_marks"
    bl_label = "Clear All Edge Marks"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # Only available in Edit Mode with a mesh object
        return context.object and context.object.type == 'MESH' and context.mode == 'EDIT_MESH'
    
    def execute(self, context):
        obj = context.object
        bm = bmesh.from_edit_mesh(obj.data)
        
        # Get bevel weight layer
        try:
            bevel_layer = bm.edges.layers.float.get('bevel_weight_edge')
            if bevel_layer is None:
                bevel_layer = bm.edges.layers.float.get('bevel_weight')
        except Exception as e:
            self.report({'WARNING'}, f"Could not access bevel weight layer: {e}")
            bevel_layer = None
        
        # Count how many edges were cleared
        cleared_count = 0
        
        # Clear seams and bevel weights for all edges
        for edge in bm.edges:
            if edge.seam or (bevel_layer and edge[bevel_layer] > 0):
                edge.seam = False
                if bevel_layer:
                    edge[bevel_layer] = 0.0
                cleared_count += 1
        
        # Update the mesh
        bmesh.update_edit_mesh(obj.data)
        
        # Select all edges before clearing freestyle marks
        bpy.ops.mesh.select_all(action='DESELECT')
        bpy.ops.mesh.select_mode(type='EDGE')
        bpy.ops.mesh.select_all(action='SELECT')
        
        # Clear freestyle marks using Blender's built-in operator
        bpy.ops.mesh.mark_freestyle_edge(clear=True)
        
        # Deselect all edges after clearing
        bpy.ops.mesh.select_all(action='DESELECT')
        
        if cleared_count > 0:
            self.report({'INFO'}, f"Cleared markings from {cleared_count} edges")
        else:
            self.report({'INFO'}, "No edge markings found to clear")
            
        return {'FINISHED'}


class OBJECT_OT_edge_mark_error(bpy.types.Operator):
    """Display error message when trying to mark edges outside of Edit Mode"""
    bl_idname = "object.edge_mark_error"
    bl_label = "Edge Marking Error"
    bl_options = {'REGISTER'}
    
    message: bpy.props.StringProperty(
        name="Message",
        description="Error message to display",
        default="Must be in Edit Mode to mark edges"
    )
    
    @classmethod
    def poll(cls, context):
        # This operator is available when NOT in Edit Mode
        return context.object and context.object.type == 'MESH' and context.mode != 'EDIT_MESH'
    
    def execute(self, context):
        self.report({'ERROR'}, self.message)
        return {'CANCELLED'}


class OBJECT_OT_clear_selected_edge_marks(bpy.types.Operator):
    """Clear seams, bevel weights, and freestyle marks for selected edges"""
    bl_idname = "object.clear_selected_edge_marks"
    bl_label = "Clear Selected Edge Marks"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # Only available in Edit Mode with a mesh object
        return context.object and context.object.type == 'MESH' and context.mode == 'EDIT_MESH'
    
    def execute(self, context):
        obj = context.active_object
        mesh = obj.data
        
        # Get current selection mode
        select_mode = context.tool_settings.mesh_select_mode
        
        # If in vertex mode or nothing selected, show error
        if select_mode[0]:  # Vertex mode
            self.report({'ERROR'}, "Please switch to edge select mode and select edges to clear")
            return {'CANCELLED'}
        
        # Create BMesh to access edge data layers
        bm = bmesh.from_edit_mesh(mesh)
        
        # Get bevel weight layer - try both possible names
        bevel_layer = bm.edges.layers.float.get('bevel_weight_edge')
        if bevel_layer is None:
            bevel_layer = bm.edges.layers.float.get('bevel_weight')
            if bevel_layer is None:
                bevel_layer = bm.edges.layers.float.new('bevel_weight_edge')
        
        # Get selected edges
        selected_edges = set()
        
        if select_mode[1]:  # Edge mode
            # Get directly selected edges
            selected_edges = {e for e in bm.edges if e.select}
        elif select_mode[2]:  # Face mode
            # Get edges of selected faces
            for face in bm.faces:
                if face.select:
                    selected_edges.update(face.edges)
        
        if not selected_edges:
            self.report({'ERROR'}, "No edges selected. Please select edges to clear")
            bm.free()
            return {'CANCELLED'}
        
        # Clear seam and bevel weights for all selected edges
        for edge in selected_edges:
            edge.seam = False  # Clear precut (seam)
            edge[bevel_layer] = 0.0  # Clear freeze (bevel weight)
        
        # Update the mesh
        bmesh.update_edit_mesh(mesh)
        bm.free()
        
        # Clear freestyle marks using Blender's built-in operator
        bpy.ops.mesh.mark_freestyle_edge(clear=True)
        
        self.report({'INFO'}, f"Selected edge marks cleared ({len(selected_edges)} edges)")
        return {'FINISHED'}


class OBJECT_OT_flip_face_normals(bpy.types.Operator):
    """Flip normals of selected faces"""
    bl_idname = "object.flip_face_normals"
    bl_label = "Flip Face"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # Only available in Edit Mode with a mesh object and in face select mode
        if not (context.object and context.object.type == 'MESH' and context.mode == 'EDIT_MESH'):
            return False
        # Check if in face select mode
        return context.tool_settings.mesh_select_mode[2]
    
    def execute(self, context):
        obj = context.active_object
        mesh = obj.data
        
        # Create BMesh
        bm = bmesh.from_edit_mesh(mesh)
        
        # Get selected faces
        selected_faces = [f for f in bm.faces if f.select]
        
        if not selected_faces:
            self.report({'ERROR'}, "Please select faces to flip")
            bm.free()
            return {'CANCELLED'}
        
        # Flip selected faces
        bmesh.ops.reverse_faces(bm, faces=selected_faces)
        
        # Update the mesh
        bmesh.update_edit_mesh(mesh)
        bm.free()
        
        self.report({'INFO'}, f"Flipped {len(selected_faces)} faces")
        return {'FINISHED'}

class VIEW3D_PT_papercraft_panel(bpy.types.Panel):
    """Panel for Unbender Tool"""
    bl_label = "Unbender"
    bl_idname = "VIEW3D_PT_unbender_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Unbender'  # This will create a new tab in the sidebar
    
    def draw(self, context):
        layout = self.layout
        
        # Help button at the top
        layout.operator("object.papercraft_help", text="Help", icon='QUESTION')
        
        # Origin Face Section
        box = layout.box()
        box.label(text="Origin Face:")
        row = box.row(align=True)
        row.operator("object.set_origin_face", text="Set")
        row.operator("object.clear_origin_face", text="Clear")
        
        # Edge Marking Section
        box = layout.box()
        box.label(text="Mark Edges:")
        
        # First row: Precut and Freeze buttons
        row = box.row(align=True)
        if context.mode == 'EDIT_MESH':
            row.operator("object.mark_edges_precut", text="Precut")
            row.operator("object.mark_edges_freeze", text="Freeze")
        else:
            row.operator("object.edge_mark_error", text="Precut").message = "Must be in Edit Mode to mark edges"
            row.operator("object.edge_mark_error", text="Freeze").message = "Must be in Edit Mode to mark edges"
        
        # Second row: Freestyle button
        row = box.row(align=True)
        if context.mode == 'EDIT_MESH':
            row.operator("object.mark_edges_freestyle", text="Freestyle")
        else:
            row.operator("object.edge_mark_error", text="Freestyle").message = "Must be in Edit Mode to mark edges"
        
        # Edge Clearing Section
        box = layout.box()
        box.label(text="Clear Edges:")
        row = box.row(align=True)
        if context.mode == 'EDIT_MESH':
            row.operator("object.clear_selected_edge_marks", text="Selected")
            row.operator("object.clear_edge_marks", text="All")
        else:
            row.operator("object.edge_mark_error", text="Selected").message = "Must be in Edit Mode to clear edges"
            row.operator("object.edge_mark_error", text="All").message = "Must be in Edit Mode to clear edges"
        
        # Face Normal Section
        box = layout.box()
        box.label(text="Face Normals:")
        row = box.row(align=True)
        if context.mode == 'EDIT_MESH':
            row.operator("object.flip_face_normals", text="Flip Face")
        else:
            row.operator("object.edge_mark_error", text="Flip Face").message = "Must be in Edit Mode to flip faces"
        
        # Unfolding Section
        box = layout.box()
        box.label(text="Unbending Method:")
        box.prop(context.window_manager, "papercraft_unfold_method", text="")
        
        # Cleanup Section
        box = layout.box()
        box.label(text="Cleanup:")
        box.prop(context.window_manager, "papercraft_cleanup_option", text="")
        
        # Unfold button
        if context.mode == 'OBJECT':
            op = box.operator("object.unfold_papercraft_multi", text="Unbend")
            op.unfold_method = context.window_manager.papercraft_unfold_method
            op.cleanup_option = context.window_manager.papercraft_cleanup_option
        else:
            box.operator("object.edge_mark_error", text="Unbend").message = "Must be in Object Mode to unfold"

class OBJECT_OT_mark_edges_freestyle(bpy.types.Operator):
    """Mark selected edges as freestyle edges"""
    bl_idname = "object.mark_edges_freestyle"
    bl_label = "Mark Edges as Freestyle"
    bl_options = {'REGISTER', 'UNDO'}
    
    @classmethod
    def poll(cls, context):
        # Only available in Edit Mode with a mesh object
        return context.object and context.object.type == 'MESH' and context.mode == 'EDIT_MESH'
    
    def execute(self, context):
        # Use Blender's built-in freestyle edge marking operator
        result = bpy.ops.mesh.mark_freestyle_edge(clear=False)
        
        if result == {'FINISHED'}:
            self.report({'INFO'}, "Marked selected edges as freestyle")
        else:
            self.report({'WARNING'}, "No edges were marked")
            
        return result

def register():
    # Register the window manager property for unfolding method
    bpy.types.WindowManager.papercraft_unfold_method = bpy.props.EnumProperty(
        name="Unfolding Method",
        description="Method to use for unfolding the mesh",
        items=[
            ('BY_PATH', "By Path", "Unfold each path independently (original method)"),
            ('TRUNK_FIRST', "Trunk First", "Unfold from trunk (origin) outward to branches"),
            ('BRANCHES_FIRST', "Branches First", "Unfold from outer branches inward to trunk")
        ],
        default='BY_PATH'
    )
    
    # Register the window manager property for cleanup options
    bpy.types.WindowManager.papercraft_cleanup_option = bpy.props.EnumProperty(
        name="Cleanup",
        description="Post-processing cleanup options",
        items=[
            ('NONE', "None", "No post-processing"),
            ('MARKS', "Marks", "Remove edge marks"),
            ('FACES', "Faces", "Remove all faces (leaving wireframe)"),
            ('ALL_BUT_FROZEN', "All but Frozen", "Remove all fold edges except frozen ones"),
            ('ALL', "All", "Remove unfolded edges")
        ],
        default='NONE'
    )
    
    # Register all classes
    bpy.utils.register_class(VIEW3D_PT_papercraft_panel)
    bpy.utils.register_class(OBJECT_OT_papercraft_help)
    bpy.utils.register_class(OBJECT_OT_clear_origin_face)
    bpy.utils.register_class(OBJECT_OT_set_origin_face)
    bpy.utils.register_class(OBJECT_OT_unfold_papercraft_multi)
    bpy.utils.register_class(OBJECT_OT_mark_edges_precut)
    bpy.utils.register_class(OBJECT_OT_mark_edges_freeze)
    bpy.utils.register_class(OBJECT_OT_clear_edge_marks)
    bpy.utils.register_class(OBJECT_OT_clear_selected_edge_marks)
    bpy.utils.register_class(OBJECT_OT_edge_mark_error)
    bpy.utils.register_class(OBJECT_OT_flip_face_normals)
    bpy.utils.register_class(OBJECT_OT_mark_edges_freestyle)

def unregister():
    # Remove the window manager property
    del bpy.types.WindowManager.papercraft_unfold_method
    del bpy.types.WindowManager.papercraft_cleanup_option
    
    # Unregister all classes
    bpy.utils.unregister_class(VIEW3D_PT_papercraft_panel)
    bpy.utils.unregister_class(OBJECT_OT_papercraft_help)
    bpy.utils.unregister_class(OBJECT_OT_clear_origin_face)
    bpy.utils.unregister_class(OBJECT_OT_set_origin_face)
    bpy.utils.unregister_class(OBJECT_OT_unfold_papercraft_multi)
    bpy.utils.unregister_class(OBJECT_OT_mark_edges_precut)
    bpy.utils.unregister_class(OBJECT_OT_mark_edges_freeze)
    bpy.utils.unregister_class(OBJECT_OT_clear_edge_marks)
    bpy.utils.unregister_class(OBJECT_OT_clear_selected_edge_marks)
    bpy.utils.unregister_class(OBJECT_OT_edge_mark_error)
    bpy.utils.unregister_class(OBJECT_OT_flip_face_normals)
    bpy.utils.unregister_class(OBJECT_OT_mark_edges_freestyle)

# For debugging in the Text Editor:
if __name__ == "__main__":
    # Safe unregistration to prevent Blender crashes on script reload
    try:
        unregister()
        print("Successfully unregistered previous version of the tool")
    except Exception as e:
        print(f"No previous version to unregister or error occurred: {e}")
    
    # Register the current version
    register()
    print("Papercraft Unfolder tool registered successfully")
    # unfold_papercraft()  # Commented out to avoid running on script load