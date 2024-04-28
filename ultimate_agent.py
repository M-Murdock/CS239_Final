# Author: Mavis, Kat, Harsh, Ju-Hung, Luoyou
import json
import socket
import numpy as np

from astar_path_planner import objs, find_item_position
from datetime import time
from queue import PriorityQueue
from utils import recv_socket_data

target_locations = {
    "entrance": [0.15, 15],
    "counter": [3, 12],
    "cart": [1, 17.5],
    "milk": [5.5, 3.5],
    "chocolate milk": [9.5, 3.5],
    "strawberry milk": [13.5, 3.5],
    "apples": [5.5, 7.5],
    "oranges": [7.5, 7.5],
    "banana": [9.5, 7.5],
    "strawberry": [11.5, 7.5],
    "raspberry": [13.5, 7.5],
    "sausage": [5.5, 11.5],
    "steak": [7.5, 11.5],
    "chicken": [11.5, 11.5],
    "ham": [13.5, 11.5],
    "brie cheese": [5.5, 15.5],
    "swiss cheese": [7.5, 15.5],
    "cheese wheel": [9.5, 15.5],
    "garlic": [5.5, 19.5],
    "leek": [7.5, 19.5],
    "red bell pepper": [9.5, 19.5],
    "carrot": [11.5, 19.5],
    "lettuce": [13.5, 19.5],
    "avocado": [5.5, 23.5],
    "broccoli": [7.5, 23.5],
    "cucumber": [9.5, 23.5],
    "yellow bell pepper": [11.5, 23.5],
    "onion": [13.5, 23.5],
    "prepared food": [18, 5],
    "fresh fish": [17.0, 15.0],
    "checkout": [4.0, 11.5],
    "exit": [-0.5, 15.6],
    "corner_1": [18, 2],
    "corner_2": [18, 22]
}


class Agent:
    def __init__(self, socket_game, env):
        self.shopping_list = env['observation']['players'][0]['shopping_list']
        self.shopping_quant = env['observation']['players'][0]['list_quant']
        self.game = socket_game
        self.map_width = 20
        self.map_height = 25
        self.obs = env['observation']
        self.cart = None
        self.basket = None
        self.player = self.obs['players'][0]
        self.last_action = "NOP"
        self.current_direction = self.player['direction']
        self.size = [0.6, 0.4]

    def step(self, action):
        # print("Sending action: ", action)
        action = "0 " + action
        self.game.send(str.encode(action))  # send action to env
        output = recv_socket_data(self.game)  # get observation from env
        # print("Observations: ", output)
        if output:
            output = json.loads(output)
            self.obs = output['observation']
            # print("Observations: ", self.obs)
            # if len(self.obs['player'][0]['carts']) > 0:
            #     self.cart = self.obs['carts'][0]
            # if len(self.obs['baskets']) > 0:
            #     self.basket = self.obs['baskets'][0]
            self.last_action = action
            self.player = self.obs['players'][0]
            # print(self.player['position'])
        return output

    def heuristic(self, a, b):
        """Calculate the Manhattan distance from point a to point b."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def overlap(self, x1, y1, width_1, height_1, x2, y2, width_2, height_2):
        return (x1 > x2 + width_2 or x2 > x1 + width_1 or y1 > y2 + height_2 or y2 > y1 + height_1)

    def objects_overlap(self, x1, y1, width_1, height_1, x2, y2, width_2, height_2):
        return self.overlap(x1, y1, width_1, height_1, x2, y2, width_2, height_2)

    def collision(self, x, y, width, height, obj):
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

        return no_overlap

    # The function will return False if the rectangle is outside the map boundaries or intersects with the object.

    def hits_wall(self, x, y):
        wall_width = 0.4
        return not (y <= 2 or y + self.size[1] >= self.map_height - wall_width or
                    x + self.size[0] >= self.map_width - wall_width)
        # return y <= 2 or y + unit.height >= len(self.map) - wall_width or \
        #        x + unit.width >= len(self.map[0]) - wall_width or (x <= wall_width and
        #                                                            not self.at_door(unit, x, y))

    def neighbors(self, point, map_width, map_height, objs):
        """Generate walkable neighboring points avoiding collisions with objects."""
        step = 0.150
        directions = [(0, step), (step, 0), (0, -step), (-step, 0)]  # Adjacent squares: N, E, S, W
        x, y = point

        results = []
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            # if 0 <= nx < map_width and 0 <= ny < map_height and all(self.collision(nx, ny, self.size[0], self.size[1], obj[]) for obj in objs):
            if 0 <= nx < map_width and 0 <= ny < map_height and all(
                    self.objects_overlap(nx, ny, self.size[0], self.size[1], obj['position'][0],
                                         obj['position'][1], obj['width'], obj['height']) for obj in
                    objs) and self.hits_wall(nx, ny):
                results.append((nx, ny))
        # print(results)
        return results

    def is_close_enough(self, current, goal, tolerance=0.15, is_item=True):
        """Check if the current position is within tolerance of the goal position."""
        if is_item is not None:
            tolerance = 0.6
            return (abs(current[0] - goal[0]) < tolerance - 0.15 and abs(current[1] - goal[1]) < tolerance + 0.05)

        else:
            return (abs(current[0] - goal[0]) < tolerance and abs(current[1] - goal[1]) < tolerance)

    def distance(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def astar(self, start, goal, objs, map_width, map_height, is_item=True):
        """Perform the A* algorithm to find the shortest path from start to goal."""
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        distance = 1000

        while not frontier.empty():

            current = frontier.get()
            # print(current, goal)
            # if distance > self.distance(current, goal):
            #     distance = self.distance(current, goal)
            #     print("getting closer: ", distance)
            if self.is_close_enough(current, goal, is_item=is_item):
                break

            for next in self.neighbors(current, map_width, map_height, objs):
                new_cost = cost_so_far[current] + 0.15  # Assume cost between neighbors is 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current

        # Reconstruct path
        if self.is_close_enough(current, goal, is_item=is_item):
            path = []
            while current:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        return None  # No path found

    # PlayerAction.NORTH: (Direction.NORTH, (0, -1), 0),
    # PlayerAction.SOUTH: (Direction.SOUTH, (0, 1), 1),
    # PlayerAction.EAST: (Direction.EAST, (1, 0), 2),
    # PlayerAction.WEST: (Direction.WEST, (-1, 0), 3)
    def from_path_to_actions(self, path):
        """Convert a path to a list of actions."""
        # if the current direction is not the same as the direction of the first step in the path, add a TURN action
        # directions = [(0, step), (step, 0), (0, -step), (-step, 0)]  # Adjacent squares: N, E, S, W
        actions = []
        cur_dir = self.current_direction

        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            if x2 > x1:
                if cur_dir != '2':
                    actions.append('EAST')
                    cur_dir = '2'
                    actions.append('EAST')
                else:
                    actions.append('EAST')
            elif x2 < x1:
                if cur_dir != '3':
                    actions.append('WEST')
                    cur_dir = '3'
                    actions.append('WEST')
                else:
                    actions.append('WEST')
            elif y2 < y1:
                if cur_dir != '0':
                    actions.append('NORTH')
                    cur_dir = '0'
                    actions.append('NORTH')
                else:
                    actions.append('NORTH')
            elif y2 > y1:
                if cur_dir != '1':
                    actions.append('SOUTH')
                    cur_dir = '1'
                    actions.append('SOUTH')
                else:
                    actions.append('SOUTH')
        return actions

    def face_item(self, goal_x, goal_y):
        x, y = self.player['position']
        cur_dir = self.current_direction
        # print("info: ", cur_dir, y, goal_y)
        if goal_y < y:
            if cur_dir != '0':
                self.step('NORTH')
                dis = abs(goal_y - y)
                while dis > 0.75:
                    self.step('NORTH')
                    if abs(dis - abs(goal_y - self.player['position'][1])) < 0.1:
                        break
                    else:
                        dis = abs(goal_y - self.player['position'][1])
                return 'NORTH'
        elif goal_y > y:
            if cur_dir != '1':
                self.step('SOUTH')
                dis = abs(goal_y - y)
                while dis > 0.75:
                    self.step('SOUTH')
                    if abs(dis - abs(goal_y - self.player['position'][1])) < 0.1:
                        break
                    else:
                        dis = abs(goal_y - self.player['position'][1])
                return 'SOUTH'

    def perform_actions(self, actions):
        """Perform a list of actions."""
        for action in actions:
            self.step(action)
        return self.obs

    def change_direction(self, direction):
        cur_dir = self.current_direction
        if direction == 'NORTH':
            if cur_dir != '0':
                self.step('NORTH')
                return 'NORTH'
        elif direction == 'SOUTH':
            if cur_dir != '1':
                self.step('SOUTH')
                return 'SOUTH'
        elif direction == 'EAST':
            if cur_dir != '2':
                self.step('EAST')
                return 'EAST'
        elif direction == 'WEST':
            if cur_dir != '3':
                self.step('WEST')
                return 'WEST'

def get_item_location(item):
    item_pos = target_locations[item]
    if item != 'prepared foods' and item != 'fresh fish':
        item_pos = target_locations[item]
    else:
        if item == 'prepared foods':
            item_pos = [18.25, 4.75]
        else:
            item_pos = [18.25, 10.75]
    if item == 'milk' or item == 'chocolate milk' or item == 'strawberry milk':
        y_offset = 3
    else:
        y_offset = 0

    item_location = [item_pos[0] + offset, item_pos[1] + y_offset]
    return item_location


def wait_in_the_corner(player, player_number):
    # takes in Agent object
    # get current location
    output = recv_socket_data(player.socket_game)  # assuming `socket_game` is the correct variable name
    state = json.loads(output)
    for player in state["observation"]["players"]:
        if player['index'] == player_number:
            location = player["position"]
            break

    # use astar to plan the path (trail ver, the agent only goes to top right corner)
    path = player.astar((location[0], location[1]),
                        (target_locations["corner_1"][0], target_locations["corner_1"][1]), objs, 20, 25)
    player.perform_actions(player.from_path_to_actions(path))

    # wait 5 sec
    time.sleep(5)

    return None


def euclidean_distance(pos1, pos2):
    # Calculate Euclidean distance between two points
    return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5


def get_next_shopping_item(state):
    shopping_list = state['observation']['players'][0]['shopping_list']
    shopping_quants = state['observation']['players'][0]['list_quant']
    user_location = state['observation']['players'][0]['position']

    quant_dict = dict(zip(shopping_list, shopping_quants))

    curr_min_distance = 999999999
    closest_item = None

    for item in shopping_list:
        coord = target_locations[item]
        distance = euclidean_distance(user_location, coord)
        if distance < curr_min_distance:
            curr_min_distance = distance
            closest_item = item

    # return closest_item
    return tuple(closest_item, quant_dict[closest_item])


def GetCurrentState(sock_game, playernumber):
    ##  example state {'NORTH,basket(4.05, 10.65)': 'NOOP', ...}
    ## 0: North 1: south 2: east 3: west

    # get the current state of the environment
    smallstate = ""
    directions = ['NORTH', 'SOUTH', 'EAST', 'WEST']
    print("getting current state")
    output = recv_socket_data(sock_game)  # get observation from env
    print("got observation")
    # print("output: ", output)
    state = json.loads(output)
    print("state: ", state)

    for player in state["observation"]["players"]:
        if player['index'] == playernumber:
            directionfacing = player["direction"]
            print("directionfacing: ", directionfacing)
            direction = directions[directionfacing]
            print("direction: ", direction)
            smallstate = smallstate + direction + ","

    if state["observation"]['baskets'] != []:
        for basket in list(state["observation"]['baskets']):
            if basket['owner'] == playernumber:
                smallstate = smallstate + "basket,"
    if state["observation"]['players'][playernumber]["curr_cart"] > 0:
        smallstate.join = "cart"
    for key in state["observation"]:
        for key2 in state["observation"][key]:
            if key == "players":
                for thisplayer in list(state["observation"][key]):
                    print("thisplayer: ", thisplayer)
                    print(thisplayer['index'])
                    if thisplayer['index'] == playernumber:
                        print("found player")
                        currentposition = str(key2["position"])
                        currentposition = currentposition.replace("[", "(")
                        currentposition = currentposition.replace("]", ")")
                        print("currentposition: ", currentposition)
                        smallstate = smallstate + currentposition

    print("smallstate: ", smallstate)
    return smallstate


def ExecutePlanToItem(preliminarypath, sock_game, playernumber):
    # the path is a dictionary of states and actions
    # get the current state from the environment
    # find out if the current state is any of the states in the path
    # if so, execute the action
    # if not, return an error
    pollingcounter = 0
    while len(preliminarypath) > 0:
        print("path length is ", len(preliminarypath))
        state = GetCurrentState(sock_game, playernumber)
        polllength = 3
        print("poll length 3")
        if pollingcounter > polllength:
            path_blocked(preliminarypath, sock_game)
            pollingcounter = 0  # reset the counter
            print("resetting")
        print("about to get the key")
        for key in preliminarypath:
            if key.find("END") != -1:  # if the key includes "END" then we are at the end of the path
                print("End of path")
                return "End of path"
            print("the key is " + str(key))
            if state == key:
                action = preliminarypath[key]
                actionok = checknorms(action)
                if actionok == True:
                    ## actually take the action in the environment
                    print("Sending action: ", action)
                    formataction = str(playernumber) + " " + action
                    print("formataction: ", formataction)
                    sock_game.send(str.encode(formataction))  # send action to env

                    preliminarypath.pop(key)
                    pollingcounter = pollingcounter + 1  # increment the stepcount
                    break
                else:
                    print("Action not allowed")
                    return "Error: action not allowed"
            else:
                print("State not in path")
                return "Error: state not in path, need to replan"


def checknorms(action):
    # use norm.py to check if the action is allowed
    # this is hard, for now
    return True

def point_in_object(point, obj):
    """
    Check if a point is within the boundary of an object.

    Parameters:
        point (tuple): The (x, y) coordinates of the point to check.
        obj (dict): An object with 'position', 'width', and 'height' indicating its boundary.

    Returns:
        bool: Returns True if the point is inside the object, False otherwise.
    """
    x, y = point
    obj_left = obj['position'][0]
    obj_right = obj['position'][0] + obj['width']
    obj_top = obj['position'][1]
    obj_bottom = obj['position'][1] + obj['height']
    if obj_left <= x <= obj_right and obj_top <= y <= obj_bottom:
        return True

    return False


def path_blocked(player_id, path, state):
    """
    Determines if a given path is blocked by any other players or carts.

    Args:
        player_id (int): ID of the player whose path is being checked.
        path (list of tuples): The path the player intends to take, as a list of (x, y) coordinates.
        state (dict): The current game state, including positions of all players and carts.

    Returns:
        bool: True if path is blocked, False otherwise.
    """
    players = state['observation']['players']
    carts = state['observation']['carts']
    player = players[player_id]
    isHoldingCart = player['curr_cart'] != -1

    for point in path:
        # Check if point is inside any other players
        for player in players:
            if player['index'] != player_id:  # Ignore self
                if point_in_object(point, player):
                    return True

        # Check if point is inside any carts
        for cart in carts:
            if cart["owner"] == player_id and isHoldingCart:
                continue  # Skip the cart being held by the player
            if point_in_object(point, cart):
                return True

    return False


def check_in_front(player_id, state, distance):
    """
    Check if there's any obstacle directly in front of the player within a specified distance
    by sampling points at every 0.1 units along the direction the player is facing.

    Args:
        player_id (int): ID of the player to check.
        state (dict): Current game state.
        distance (float): Distance to check in front of the player.

    Returns:
        bool: True if there's an obstacle in front, False otherwise.
    """
    players = state['observation']['players']
    carts = state['observation']['carts']
    player = players[player_id]
    isHoldingCart = player['curr_cart'] != -1

    # Player properties
    x, y = player['position']
    x += player['width'] / 2
    direction = player['direction']
    step = 0.1

    # Generate points along the direction
    points = []
    if direction == 0:  # North
        points = [(x, y - i * step) for i in range(int(distance / step) + 1)]
    elif direction == 1:  # South
        points = [(x, y + i * step) for i in range(int(distance / step) + 1)]
    elif direction == 2:  # East
        points = [(x + i * step, y) for i in range(int(distance / step) + 1)]
    elif direction == 3:  # West
        points = [(x - i * step, y) for i in range(int(distance / step) + 1)]

    # Check each point for collision with players
    for p in players:
        if p['index'] != player_id:
            for point in points:
                if point_in_object(point, p):
                    return True

    # Check each point for collision with carts
    for cart in carts:
        if cart["owner"] == player_id and isHoldingCart:
            continue  # Skip the cart being held by the player
        for point in points:
            if point_in_object(point, cart):
                return True

    return False


def obstacle_in_aisle(player_id, state, shopping_item, direction):
    """
    Determine if the aisle above or below the shelf with the specified item is blocked.

    Args:
        player_id (int): ID of the player to check around.
        state (dict): Current game state.
        shopping_item (str): The shopping item to locate which aisle.
        direction (str): 'above' or 'below' to specify which aisle to check.

    Returns:
        bool: True if the aisle is blocked, False otherwise.
    """
    shelves = [
        ["milk", "chocolate milk", "strawberry milk"],
        ["apples", "oranges", "banana", "strawberry", "raspberry"],
        ["sausage", "steak", "chicken", "ham"],
        ["brie cheese", "swiss cheese", "cheese wheel"],
        ["garlic", "leek", "red bell pepper", "carrot", "lettuce"],
        ["avocado", "broccoli", "cucumber", "yellow bell pepper", "onion"]
    ]

    aisle_y_coords = [
        (2.55, 5.1),  # Aisle 1
        (6.5, 9.05),  # Aisle 2
        (10.6, 12.9),  # Aisle 3
        (14.5, 17),  # Aisle 4
        (18.65, 21.1),  # Aisle 5
        (22.5, 24)  # Aisle 6
    ]

    # Find which shelf the item is on
    shelf_index = next(i for i, shelf in enumerate(shelves) if shopping_item in shelf)

    # Determine the aisles to check based on the shelf index and direction
    if direction == "above":
        if shelf_index == 0:
            aisle_to_check = aisle_y_coords[shelf_index]
        else:
            aisle_to_check = aisle_y_coords[shelf_index - 1]
    elif direction == "below":
        aisle_to_check = aisle_y_coords[shelf_index]

    x_start, x_end = 5.25, 15.25
    y_start, y_end = aisle_to_check

    # Rectangle for the aisle
    aisle_rect = {'position': (x_start, y_start), 'width': x_end - x_start, 'height': y_end - y_start}

    # Check for obstacles in the aisle
    players = state['observation']['players']
    carts = state['observation']['carts']
    player = players[player_id]
    isHoldingCart = player['curr_cart'] != -1

    # Check players
    for p in players:
        if p['index'] == player_id:
            continue  # Skip the player themselves
        player_point = (p['position'][0], p['position'][1])
        if point_in_object(player_point, aisle_rect):
            return True

    # Check carts
    for cart in carts:
        if cart["owner"] == player_id and isHoldingCart:
            continue  # Skip the cart held by the player
        cart_point = (cart['position'][0], cart['position'][1])
        if point_in_object(cart_point, aisle_rect):
            return True

    return False

if __name__ == "__main__":
    action_commands = ['NOP', 'NORTH', 'SOUTH', 'EAST', 'WEST', 'TOGGLE_CART', 'INTERACT']

    print("action_commands: ", action_commands)

    # Connect to Supermarket
    HOST = '127.0.0.1'
    PORT = 9000
    sock_game = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_game.connect((HOST, PORT))
    sock_game.send(str.encode("0 RESET"))  # reset the game
    print("Which player number are you?")
    playernumber = int(input())
    state = recv_socket_data(sock_game)
    game_state = json.loads(state)
    shopping_list = game_state['observation']['players'][playernumber]['shopping_list']
    shopping_quant = game_state['observation']['players'][playernumber]['list_quant']
    player = Agent(socket_game=sock_game, env=game_state)
    cartReturns = [2, 18.5]
    basketReturns = [3.5, 18.5]
    registerReturns_1 = [2, 4.5]
    registerReturns_2 = [2, 9.5]
    print(shopping_list)

    offset = 1

    # shopping_list = ['fresh fish', 'prepared foods']

    if 0 < len(shopping_list) <= 6:
        print("go for basket (item less than 6)")
        player.perform_actions(
            player.from_path_to_actions(player.astar((player.player['position'][0], player.player['position'][1]),
                                                     (basketReturns[0], basketReturns[1]), objs, 20, 25)))
        player.face_item(basketReturns[0], basketReturns[1])
        player.step('INTERACT')
        player.step('INTERACT')
        for item in shopping_list:
            if obstacle_in_aisle(playernumber, game_state, item, "above") is False:
                y_offset = 0
                print("go for item: ", item)
                item_pos = find_item_position(game_state, item)
                if item != 'prepared foods' and item != 'fresh fish':
                    item_pos = find_item_position(game_state, item)
                    # item = update_position_to_center(item_pos)
                else:
                    if item == 'prepared foods':
                        item_pos = [18.25, 4.75]
                    else:
                        item_pos = [18.25, 10.75]
                if item == 'milk' or item == 'chocolate milk' or item == 'strawberry milk':
                    y_offset = 3
                # print("item_pos: ", item_pos)
                path = player.astar((player.player['position'][0], player.player['position'][1]),
                                    (item_pos[0] + offset, item_pos[1] + y_offset), objs, 20, 25)
                if path == None:
                    continue
                player.perform_actions(player.from_path_to_actions(path))
                player.face_item(item_pos[0] + offset, item_pos[1])
                # player.face_item(item_pos[0] + offset, item_pos[1])
                for i in range(shopping_quant[shopping_list.index(item)]):
                    player.step('INTERACT')
                    if item == 'prepared foods' and item == 'fresh fish':
                        player.step('INTERACT')
            else:
                shopping_list.append(shopping_list.pop(0))
                shopping_quant.append(shopping_quant.pop(0))

            # print(player.obs)
        # print(player.obs['players'][0]['shopping_list'])

        # go to closer register
        if player.player['position'][1] < 7:
            path = player.astar((player.player['position'][0], player.player['position'][1]),
                                (registerReturns_1[0] + offset, registerReturns_1[1]), objs, 20, 25)
            if path == None:
                print("no path to register")
            player.perform_actions(player.from_path_to_actions(path))
            # player.face_item(registerReturns_1[0] + offset, registerReturns_1[1])
        else:
            path = player.astar((player.player['position'][0], player.player['position'][1]),
                                (registerReturns_2[0] + offset, registerReturns_2[1]), objs, 20, 25)
            if path == None:
                print("no path to register")
            player.perform_actions(player.from_path_to_actions(path))
            # player.face_item(registerReturns_2[0] + offset, registerReturns_2[1])

        player.step('INTERACT')
        player.step('INTERACT')
        player.step('INTERACT')
        print(player.obs)
