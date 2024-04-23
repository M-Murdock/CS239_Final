# Author: Mavis, Kat, Harsh, Ju-Hung, Luoyou

import json
import random
import socket

target_locations = {
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
    "fresh fish": [ 17.0, 15.0],
    "checkout": [4.0, 11.5],
    "exit": [-0.5, 15.6],
    "corner_1": [18, 2],
    "corner_2": [18, 22]
}

def get_item_location(item):
    if item in target_locations:
        return target_locations[item]
    else:
        return None

def wait_in_the_corner(socket_game, player_number):
    # get current location
    output = recv_socket_data(sock_game)  # get observation from env
    state = json.loads(output)
    for key in state["observation"]:
        for key2 in state["observation"][key]:
            if key == "players":
                if key['index'] == player_number:
                    location = key2["position"]

    # use astar to plan the path
    # wait 5 sec
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
    # get the current state of the environment
    smallstate = ""
    output = recv_socket_data(sock_game)  # get observation from env
    state = json.loads(output)
    for key in state["observation"]: 
     for key2 in state["observation"][key]: 
        if key == "players":
            if key['index'] == ourplayernumber:
                currentposition = key2["position"]
                basket = key2["baskets"]
                carts = key2["carts"]
    smallstate.append(basket)
    smallstate.append(carts)
    smallstate.append(currentposition)
    return smallstate


def ExecutePlanToItem(preliminarypath, sock_game): 
    # the path is a dictionary of states and actions
    # get the current state from the environment
    # find out if the current state is any of the states in the path
    # if so, execute the action
    # if not, return an error
    pollingcounter = 0
    while len(preliminarypath) > 0:
        state = GetCurrentState()
        polllength = 3
        if pollingcounter > polllength:
            CheckPathBlocked(preliminarypath, sock_game)
            pollingcounter = 0 # reset the counter
        for key in preliminarypath:
            if state == key:
                action = preliminarypath[key]
                print("Sending action: ", action)
                sock_game.send(str.encode(action))  # send action to env
                preliminarypath.pop(key)
                pollingcounter = pollingcounter + 1 # increment the stepcount
                break
            else:   
                return "Error: state not in path, need to replan"
        

    