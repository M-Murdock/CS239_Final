# Code is adapted from astar_path_planner-1 by Hang Yu

from utils import recv_socket_data
import json
from queue import PriorityQueue
import socket

from utils import recv_socket_data


class PathPlanner:
    def __init__(self):

        self.map_width = 20
        self.map_height = 25
        self.size = [0.6, 0.4] # by default, the agent isn't holding anything
        
        self.cartReturns = [2, 18.5]
        self.basketReturns = [3.5, 18.5]
        self.registerReturns_1 = [2, 4.5]
        self.registerReturns_2 = [2, 9.5]
        
        self.game_state = None
        self.details = ""

        self.objs = [
        {'height': 2.5, 'width': 3, 'position': [0.2, 4.5], 're_centered_position': [2.125, 5.75]},
        {'height': 2.5, 'width': 3, 'position': [0.2, 9.5], 're_centered_position': [2.125, 10.75]},
        {'height': 1, 'width': 2, 'position': [5.5, 1.5], 're_centered_position': [6.5, 2]},
        {'height': 1, 'width': 2, 'position': [7.5, 1.5], 're_centered_position': [8.5, 2]},
        {'height': 1, 'width': 2, 'position': [9.5, 1.5], 're_centered_position': [10.5, 2]},
        {'height': 1, 'width': 2, 'position': [11.5, 1.5], 're_centered_position': [12.5, 2]},
        {'height': 1, 'width': 2, 'position': [13.5, 1.5], 're_centered_position': [14.5, 2]},
        {'height': 1, 'width': 2, 'position': [5.5, 5.5], 're_centered_position': [6.5, 6]},
        {'height': 1, 'width': 2, 'position': [7.5, 5.5], 're_centered_position': [8.5, 6]},
        {'height': 1, 'width': 2, 'position': [9.5, 5.5], 're_centered_position': [10.5, 6]},
        {'height': 1, 'width': 2, 'position': [11.5, 5.5], 're_centered_position': [12.5, 6]},
        {'height': 1, 'width': 2, 'position': [13.5, 5.5], 're_centered_position': [14.5, 6]},
        {'height': 1, 'width': 2, 'position': [5.5, 9.5], 're_centered_position': [6.5, 10]},
        {'height': 1, 'width': 2, 'position': [7.5, 9.5], 're_centered_position': [8.5, 10]},
        {'height': 1, 'width': 2, 'position': [9.5, 9.5], 're_centered_position': [10.5, 10]},
        {'height': 1, 'width': 2, 'position': [11.5, 9.5], 're_centered_position': [12.5, 10]},
        {'height': 1, 'width': 2, 'position': [13.5, 9.5], 're_centered_position': [14.5, 10]},
        {'height': 1, 'width': 2, 'position': [5.5, 13.5], 're_centered_position': [6.5, 14]},
        {'height': 1, 'width': 2, 'position': [7.5, 13.5], 're_centered_position': [8.5, 14]},
        {'height': 1, 'width': 2, 'position': [9.5, 13.5], 're_centered_position': [10.5, 14]},
        {'height': 1, 'width': 2, 'position': [11.5, 13.5], 're_centered_position': [12.5, 14]},
        {'height': 1, 'width': 2, 'position': [13.5, 13.5], 're_centered_position': [14.5, 14]},
        {'height': 1, 'width': 2, 'position': [5.5, 17.5], 're_centered_position': [6.5, 18]},
        {'height': 1, 'width': 2, 'position': [7.5, 17.5], 're_centered_position': [8.5, 18]},
        {'height': 1, 'width': 2, 'position': [9.5, 17.5], 're_centered_position': [10.5, 18]},
        {'height': 1, 'width': 2, 'position': [11.5, 17.5], 're_centered_position': [12.5, 18]},
        {'height': 1, 'width': 2, 'position': [13.5, 17.5], 're_centered_position': [14.5, 18]},
        {'height': 1, 'width': 2, 'position': [5.5, 21.5], 're_centered_position': [6.5, 22]},
        {'height': 1, 'width': 2, 'position': [7.5, 21.5], 're_centered_position': [8.5, 22]},
        {'height': 1, 'width': 2, 'position': [9.5, 21.5], 're_centered_position': [10.5, 22]},
        {'height': 1, 'width': 2, 'position': [11.5, 21.5], 're_centered_position': [12.5, 22]},
        {'height': 1, 'width': 2, 'position': [13.5, 21.5], 're_centered_position': [14.5, 22]},
        {'height': 6, 'width': 0.7, 'position': [1, 18.5], 're_centered_position': [1.35, 21.5]},
        {'height': 6, 'width': 0.7, 'position': [2, 18.5], 're_centered_position': [2.35, 21.5]},
        {'height': 0.8, 'width': 0.8, 'position': [3.5, 18.5], 're_centered_position': [4.15, 19.4]},
        {'height': 2.25, 'width': 1.5, 'position': [18.25, 4.75], 're_centered_position': [19.125, 5.875]},
        {'height': 2.25, 'width': 1.5, 'position': [18.25, 10.75], 're_centered_position': [19.125, 11.875]}
    ]
        
    
    def _heuristic(self, a, b):
        """Calculate the Manhattan distance from point a to point b."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _collision(self, x, y, width, height, obj):
        """
        Check if a rectangle defined by (x, y, width, height) does NOT intersect with an object
        and ensure the rectangle stays within the map boundaries.

        Parameters:
            x (float): The x-coordinate of the rectangle's top-left corner.
            y (float): The y-coordinate of the rectangle's top-left corner.
            width (float): The width of the rectangle.
            height (float): The height of the rectangle.
            obj (dict): An object with 'position', 'width', and 'height'.

        Returns:
            bool: Returns True if there is NO collision (i.e., no overlap) and the rectangle is within map boundaries,
                False if there is a collision or the rectangle goes outside the map boundaries.
        """
        # Define map boundaries
        min_x = 0.5
        max_x = 24
        min_y = 2.5
        max_y = 19.5

        # Calculate the boundaries of the rectangle
        rectangle = {
            'northmost': y,
            'southmost': y + height,
            'westmost': x,
            'eastmost': x + width
        }
        
        # Ensure the rectangle is within the map boundaries
        if not (min_x <= rectangle['westmost'] and rectangle['eastmost'] <= max_x and
                min_y <= rectangle['northmost'] and rectangle['southmost'] <= max_y):
            return False  # The rectangle is out of the map boundaries

        # Calculate the boundaries of the object
        obj_box = {
            'northmost': obj['position'][1],
            'southmost': obj['position'][1] + obj['height'],
            'westmost': obj['position'][0],
            'eastmost': obj['position'][0] + obj['width']
        }

        # Check if there is no overlap using the specified cardinal bounds
        no_overlap = not (
            (obj_box['northmost'] <= rectangle['northmost'] <= obj_box['southmost'] or
            obj_box['northmost'] <= rectangle['southmost'] <= obj_box['southmost']) and (
                (obj_box['westmost'] <= rectangle['westmost'] <= obj_box['eastmost'] or
                obj_box['westmost'] <= rectangle['eastmost'] <= obj_box['eastmost'])
            )
        )
        
        return no_overlap # The function will return False if the rectangle is outside the map boundaries or intersects with the object.

    def _hits_wall(self, x, y):
        wall_width = 0.4
        return not (y <= 2 or y + self.size[1] >= self.map_height - wall_width or \
                x + self.size[0] >= self.map_width - wall_width) 
        
    def _neighbors(self, point, map_width, map_height):
        """Generate walkable neighboring points avoiding collisions with objects."""
        step = 0.150
        directions = [(0, step), (step, 0), (0, -step), (-step, 0)]  # Adjacent squares: N, E, S, W
        x, y = point
        
        results = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_width and 0 <= ny < map_height and all(self._objects_overlap(nx, ny, self.size[0], self.size[1], obj['position'][0], obj['position'][1], obj['width'], obj['height']) for obj in self.objs) and  self._hits_wall( nx, ny):
                results.append((nx, ny))
        return results

    def _is_close_enough(self, current, goal, tolerance=0.15, is_item = True):
        """Check if the current position is within tolerance of the goal position."""
        if is_item is not None:
            tolerance = 0.6
            return (abs(current[0] - goal[0]) < tolerance - 0.15  and abs(current[1] - goal[1]) < tolerance +0.05 )

        else:
            return (abs(current[0] - goal[0]) < tolerance and abs(current[1] - goal[1]) < tolerance)
    """
    Takes a goal and performs A* algorithm to find shortest path from start to finish
    """
    def _astar(self, start, goal, is_item = True):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        print("\n\n\n goal: ", goal )
        
        while not frontier.empty():

            current = frontier.get()
            if self._is_close_enough(current, goal, is_item=is_item):
                break

            for next in self._neighbors(current, self.map_width, self.map_height):
                new_cost = cost_so_far[current] + 0.15  # Assume cost between neighbors is 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self._heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current

        # Reconstruct path
        if self._is_close_enough(current, goal, is_item=is_item):
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        return None  # No path found

    def _overlap(self, x1, y1, width_1, height_1, x2, y2, width_2, height_2):
        return  (x1 > x2 + width_2 or x2 > x1 + width_1 or y1 > y2 + height_2 or y2 > y1 + height_1)

    def _objects_overlap(self, x1, y1, width_1, height_1, x2, y2, width_2, height_2):
        return self._overlap(x1, y1, width_1, height_1, x2, y2, width_2, height_2)
    
    """
    Takes a path (given by A*) and converts it to a list of actions
    """
    def _get_actions(self, path, goal):
        # if the current direction is not the same as the direction of the first step in the path, add a TURN action
        # directions = [(0, step), (step, 0), (0, -step), (-step, 0)]  # Adjacent squares: N, E, S, W
        actions = []
        
        # navigate to the goal
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            if x2 > x1:
                actions.append('EAST')
            elif x2 < x1:
                actions.append('WEST')
            elif y2 < y1:
                actions.append('NORTH')
            elif y2 > y1:
                actions.append('SOUTH')
                
        # face the goal
        x, y = goal
        if path[-1][1] < y:
            if not actions[-1] == 'NORTH':
                actions.append('NORTH')
        elif path[-1][1] > y:
            if not actions[-1] == 'SOUTH':
                actions.append('SOUTH')
        else: 
            actions.append("NOOP")
        
        return actions
    
    # -----------------------------------------
    """
    Using actions and path, build a dictionary of states and actions
    """
    def _build_state_action_dict(self, actions, path, last_action="NOOP"):
        print(len(actions))
        print(len(path))
        
        # keep track of the agent's direction
        # (0=north, 1=south, 2=east, 3=west)
        directions = ['NORTH', 'SOUTH', 'EAST', 'WEST']
        last_direction = directions[self.game_state['observation']['players'][0]['direction']]
        last_position = None 
        
        state_action_dict = {}
        for i in range(0, len(path)):
            state_action_dict[last_direction + "," + self.details + "" + str((round(path[i][0], 3), round(path[i][1], 3)))] = actions[i]
   # --------
            last_position = (round(path[i][0], 3), round(path[i][1], 3))
            
            if (actions[i] == 'NORTH') or (actions[i] == 'SOUTH') or (actions[i] == 'EAST') or (actions[i] == 'WEST'): 
                last_direction = actions[i]
            state_action_dict[actions[i] + "," + self.details + "" + str((round(path[i][0], 3), round(path[i][1], 3)))] = actions[i]
        
        # Turn to face the goal (but only if we're not already facing the goal!!)
        goal_pos = path[-1]
        # find whether we're facing the goal 
        if last_position[0] < goal_pos[0]: # if x < goal_x then we should face east
            # if already facing east, then do nothing 
            if not (last_direction == 'EAST'):
                state_action_dict[last_direction + "," + self.details + "" + str((round(path[i][0], 3), round(path[i][1], 3)))] = 'EAST'
        elif last_position[1] < goal_pos[1]: # elif y < goal_y then we should face south
            # if already facing south, do nothing
            if not (last_direction == 'SOUTH'):
                state_action_dict[last_direction + "," + self.details + "" + str((round(path[i][0], 3), round(path[i][1], 3)))] = 'SOUTH'
        elif last_position[0] > goal_pos[0]: # elif x > goal_x then we should face west
            # if already facing west, do nothing 
            if not (last_direction == 'WEST'):
                state_action_dict[last_direction + "," + self.details + "" + str((round(path[i][0], 3), round(path[i][1], 3)))] = 'WEST'
        elif last_position[1] > goal_pos[1]: # elif y > goal_y then we should face north
            # if already facing north, do nothing
            if not (last_direction == 'NORTH'):
                state_action_dict[last_direction + "," + self.details + "" + str((round(path[i][0], 3), round(path[i][1], 3)))] = 'NORTH'
            
        # if not, turn to face the goal 
        state_action_dict["END," + last_direction + "," + self.details + "" + str((round(path[i][0], 3), round(path[i][1], 3)))] = last_action
    # --------
    
        return state_action_dict
    
    # returns the y coordinate of the midpoint of the agent's current aisle
    # this is used for determining where to leave the cart when getting something off the shelf
    def _get_aisle_midpoint(shelf, goal):
        y = goal[1]
        
        # top aisle
        if y <= 5.5:
            mid = 3.5
            
        elif y <= 9.5:
            mid = 7.5

        elif y <= 13.5:
            mid = 11.5
            
        elif y <= 17.5:
            mid = 15.5
            
        elif y <= 21.5:
            mid = 18.5
        
        else:
            mid = 23.5
            
        return (goal[0], mid)
            
    # creates state-action dict to pick up basket or cart
    def grab_cart_or_basket(self, env, kind="basket"):  
        self.game_state = env 
            
        if kind == "cart":
            goal_x = self.cartReturns[0] + .5
            goal_y = self.cartReturns[1]
        else:
            goal_x = self.basketReturns[0] 
            goal_y = self.basketReturns[1] 
            goal_x = self.basketReturns[0] + .5
            goal_y = self.basketReturns[1]
            
        
        path_dict = self._goto(goal=(goal_x, goal_y), last_action="INTERACT")
        
        directions = ['NORTH', 'SOUTH', 'EAST', 'WEST']
        last_direction = directions[self.game_state['observation']['players'][0]['direction']]
        pos = (self.game_state['observation']['players'][0]['position'][0], self.game_state['observation']['players'][0]['position'][1])
        # if not facing south, then face south
        if not last_direction == 'SOUTH':
            path_dict[last_direction + "," + self.details + "" + str((round(pos[i][0], 3), round(pos[i][1], 3)))] = "SOUTH"
        
        return path_dict
    
    def checkout(self, env):
        self.game_state = env
        start = (self.game_state['observation']['players'][0]['position'][0], self.game_state['observation']['players'][0]['position'][1])
        
        if start[1] < 7:
            goal_x = self.registerReturns_1[0] + 1
            goal_y = self.registerReturns_1[1]
        else:
            goal_x = self.registerReturns_2[0] + 1
            goal_y = self.registerReturns_2[1]
  
        path_dict = self._goto(goal=(goal_x, goal_y), last_action='INTERACT')
 
        return path_dict
        
    def _goto(self, start=None, goal=None, last_action="NOOP"):
        if goal == None:
            return 
        if start == None:
            start = (self.game_state['observation']['players'][0]['position'][0], self.game_state['observation']['players'][0]['position'][1])
        
        path = self._astar(start, goal, is_item = True) # get xy coordinates
        if path == None:
            return None
        print("-----\n LAST ACTIONS = ", str(path[-1]), ", ", str(path[-2]), "\n --------")
        actions = self._get_actions(path, goal) # get action to take from each xy position
        if actions == None:
            return None  
        
        state_act_dict = self._build_state_action_dict(actions, path, last_action=last_action)
        
        return state_act_dict  
        
        
    def _get_path_with_cart(self, goal):
        self.size = [1.1, 1.15]
        self.details = "cart"
        # get close to the shelf and leave the cart
        leave_cart_loc = self._get_aisle_midpoint(goal)
        state_act_dict = self._goto(goal=leave_cart_loc, last_action='TOGGLE_CART')
        
        # grab the item
        self.size = [0.6, 0.4]
        self.details = ""
        try:
            state_act_dict.update(self._goto(start=leave_cart_loc, goal=goal, last_action='INTERACT'))
        except:
            return None 
        
        # return to cart
        try:
            state_act_dict.update(self._goto(start=goal, goal=leave_cart_loc, last_action='TOGGLE_CART')) # drop the item in the cart
        except:
            return None
        self.size = [1.1, 1.15]
        self.details = "cart"
        
        return state_act_dict
    
    # -----------------------------------------
    """
    Public function for generating path from start to goal. Returns the path as a dictionary of states and actions
    """
    def get_path(self, env, goal, last_action="INTERACT", has_basket=True, has_cart=False, grabbing_item=True):   
        print("getting path to goal: ", goal)
        # get observations from the environment
        self.game_state = env
        if has_basket:
            self.details = "basket"
        elif has_cart:
            self.details = "cart"
        
        # The plan will be different if we have a cart
        if self.details == "cart":
            if grabbing_item:
                path_dict = self._get_path_with_cart(goal)
            else:
                self.size = [1.1, 1.15]
                path_dict = self._goto(goal=goal, last_action=last_action)
                
        else:
            path_dict = self._goto(goal=goal, last_action=last_action)
            
        return path_dict
            




# if __name__ == "__main__":
#     action_commands = ['NOP', 'NORTH', 'SOUTH', 'EAST', 'WEST', 'TOGGLE_CART', 'INTERACT']
    
#     # Connect to Supermarket
#     HOST = '127.0.0.1'
#     PORT = 9000
#     sock_game = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     sock_game.connect((HOST, PORT))
#     sock_game.send(str.encode("0 RESET"))  # reset the game
#     state = recv_socket_data(sock_game)
#     game_state = json.loads(state)
#     shopping_list = game_state['observation']['players'][0]['shopping_list']
#     shopping_quant = game_state['observation']['players'][0]['list_quant']
#     player = PathPlanner()
    
#     offset = 1
        
#     item = "red bell pepper" #shopping_list[0]
    
#     print("go for item: ", item)
#     item_pos = find_item_position(game_state, item)
#     print(item_pos)
#     path = player.get_path(game_state, (item_pos[0] + offset, item_pos[1]), has_cart=True, grabbing_item=False)
#     # path = player.checkout(game_state)
#     # path = player.grab_cart_or_basket(game_state)
#     print(path)
    
    