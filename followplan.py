# Author: Mavis, Kat, Harsh, Ju-Hung, Luoyou

import json
import random
import socket
import time
from utils import recv_socket_data
#from astar_path_planner import *
from navigation_utils import *

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
    return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5


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
##FIXME get direction also 
##  example state {'NORTH,basket(4.05, 10.65)': 'NOOP', ...}
## 0: North 1: south 2: east 3: west

    # get the current state of the environment
    smallstate = ""
    directions = ['NORTH', 'SOUTH', 'EAST', 'WEST']
    output = recv_socket_data(sock_game)  # get observation from env
    state = json.loads(output)
    directionfacing = state["observation"]['players'][playernumber]["direction"]
    direction = directions[directionfacing]  
    if state["observation"]['baskets'] != []:
        for basket in list(state["observation"]['baskets']):
            if basket['owner'] == playernumber:
                smallstate = smallstate + "basket"       
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
                    print("currentposition: ", currentposition)
                    smallstate = smallstate + currentposition


    print("smallstate: ", smallstate)                
    return smallstate


def ExecutePlanToItem(preliminarypath, sock_game): 
    # the path is a dictionary of states and actions
    # get the current state from the environment
    # find out if the current state is any of the states in the path
    # if so, execute the action
    # if not, return an error
    pollingcounter = 0
    while len(preliminarypath) > 0:
        # print("path length is " + str(preliminarypath))
        state = GetCurrentState(sock_game, 0) # NOTE: this is only for player 0
        polllength = 3
        print("poll length 3")
        if pollingcounter > polllength:
            path_blocked(preliminarypath, sock_game)
            pollingcounter = 0 # reset the counter
            print("resetting")
        print("about to get the key")
        for key in preliminarypath:
            print("the key is " + str(key))
            if state == key:
                action = preliminarypath[key]
                actionok = checknorms(action)
                if actionok == True:
                    ## actually take the action in the environment
                    print("Sending action: ", action)
                    sock_game.send(str.encode(action))  # send action to env
                    preliminarypath.pop(key)
                    pollingcounter = pollingcounter + 1 # increment the stepcount
                    break
                else:
                    return "Error: action not allowed"
            else:   
                return "Error: state not in path, need to replan"
        
def checknorms(action):
    # use norm.py to check if the action is allowed
    # this is hard, for now
    return True
    
    