import numpy as np
import copy


def main(maze):
  end_position = find_exit_target(maze)
  intial_position = (0, 0)
  path = a_star_finder(maze, intial_position, end_position)
  resolved_maze = mark_path(maze, path)

  print(path)
  print("="*100)
  plot_maze(maze)
  print("="*100)
  plot_maze(resolved_maze)

def find_exit_target(maze):
  '''
    Find the exit target in the maze.

    Args:
      maze: 2D array of integers.

    Returns:
      Tuple of the exit target position.
  '''
  np_matrix = np.array(maze)
  target_coordenates_np = np.where(np_matrix == 1)
  end_position = (target_coordenates_np[0][0], target_coordenates_np[1][0])
  return end_position

def a_star_finder(maze, start_position, target_exit):
  '''
    Find the path from the start position to the target exit.

    Args:
      maze: 2D array of integers.
      start_position: Tuple of the start position.
      target_exit: Tuple of the target exit.

    Returns:
      List of tuples with the path.
  '''
  start_node = create_node(None, start_position)
  end_node = create_node(None, target_exit)
  open_list, closed_list = [], []
  open_list.append(start_node)

  while len(open_list) > 0:
    current_node, current_index = get_lowest_f_cost_node(open_list)
    open_list.pop(current_index)
    closed_list.append(current_node)

    path = get_path(current_node, end_node)
    if path: return path[::-1]

    children = get_children(current_node, maze)
    for child in children:
      if child in closed_list:
        continue

      child = calculate_child_node_cost(child, end_node, current_node)
      for open_node in open_list:
        if child == open_node and child["g_cost"] > open_node["g_cost"]:
          continue
      
      open_list.append(child)

def create_node(parent, position, g_cost=0, h_cost=0):
  '''
    Create a node with the parent the parent node and the cost properties.

    Args:
      parent: Node.
      position: Tuple of the position.
      g_cost: Integer.
      h_cost: Integer.

    Returns:
      Node
  '''
  node = {
    "parent": parent,
    "position": position,
    "g_cost": g_cost,
    "h_cost": h_cost,
    "f_cost": g_cost + h_cost
  }
  return copy.deepcopy(node)

def get_lowest_f_cost_node(open_list):
  '''
    Get the node with the lowest f cost.

    Args:
      open_list: List of nodes.

    Returns:
      Tuple of the node and the index of the node.
  '''
  lowest_f_cost_node = open_list[0]
  for node in open_list:
    if node["f_cost"] < lowest_f_cost_node["f_cost"]:
      lowest_f_cost_node = node
  return lowest_f_cost_node, open_list.index(lowest_f_cost_node)

def get_path(current_node, end_node):
  '''
    Get the path from the current node to the end node.

    Args:
      current_node: Node.
      end_node: Node.

    Returns:
      List of tuples with the path.
  '''
  path = []
  if current_node["position"] == end_node["position"]:
    while current_node["parent"] is not None:
      path.append(current_node["position"])
      current_node = current_node["parent"]
  return path 

def get_children(current_node, maze):
  '''
    Get the children of the current node.

    Args:
      current_node: Node.
      maze: 2D array of integers.

    Returns:
      List of nodes.
  '''
  children = []
  neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0)]
  for neighbor in neighbors:
    new_node = get_valid_node(current_node, neighbor, maze)
    if new_node:
      children.append(new_node)
    else:
      continue
  return children

def get_valid_node(current_node, neighbor, maze):
  '''
    Get the valid node from the current node and the neighbor.

    Args:
      current_node: Node.
      neighbor: Tuple of the neighbors nodes.
      maze: 2D array of integers.

    Returns:
      Node.
  '''
  try:
    new_position = (
      current_node["position"][0] + neighbor[0], 
      current_node["position"][1] + neighbor[1])
    if maze[new_position[0]][new_position[1]] > 0:
      return create_node(current_node, new_position)
  except IndexError:
    return None
  
def calculate_child_node_cost(child, end_node, current_node):
  '''
    Calculate the cost of the child node.

    Args:
      child: Node.
      end_node: Node.
      current_node: Node.

    Returns:
      Node.
  '''
  child["g_cost"] = current_node["g_cost"] + 1
  child["h_cost"] = calculate_h_cost(child, end_node)
  child["f_cost"] = child["g_cost"] + child["h_cost"]
  return child

def calculate_h_cost(child, end_node):
  '''
    Calculate the h cost of the child node.

    Args:
      child: Node.
      end_node: Node.

    Returns:
      Integer H cost.
  '''
  h_cost = abs(child["position"][0] - end_node["position"][0]) + abs(child["position"][1] - end_node["position"][1])
  return h_cost

def mark_path(maze, path):
  '''
    Mark the path in the maze.

    Args:
      maze: a matrix 2D array of integers.
      path: List of tuples with the path.

    Returns:
      Matrix: 2D array of integers.
  '''
  new_maze = copy.deepcopy(maze)
  for position in path:
    new_maze[position[0]][position[1]] = "S"
  return new_maze

def plot_maze(maze):
  '''
    Plot the maze.

    Args:
      maze: 2D array of integers.
  '''
  for row in maze:
    for cell in row:
      if cell == "S":
        print("_", end=" ")
        continue
      if cell >= 0:
        print("O", end=" ")
      else:
        print("X", end=" ")
    print()

maze = [
  [26, -1, -1, -1, 12, 11, 10, 9, 10, 11, 12],
  [25, -1,  0,  0, -1, 12, -1, 8, -1, -1, 13],
  [24, 25, -1,  0, -1, 13, -1, 7,  6,  5, -1],
  [23, -1, 21, -1, 15, 14, 15, -1, -1, 4,  3],
  [22, 21, 20, -1, 16, -1, 16, 17, 18, -1, 2],
  [23, -1, 19, 18, 17, 18, 17, 18, -1,  2, 1]
]

main(maze)
