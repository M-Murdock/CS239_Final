import json
import random
import socket

from env import SupermarketEnv
from utils import recv_socket_data
import followplan   
import preliminary_path

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

HOST = '127.0.0.1'
PORT = 9000
sock_game = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_game.connect((HOST, PORT))
sock_game.send(str.encode("0 RESET"))  # reset the game

print("Which player number are you?")
playernumber = int(input())
output = recv_socket_data(sock_game)
game_state = json.loads(output)
print("got observations for the first time")

""" steps = 0
while steps < 10: 
    action = "0 " + "SOUTH"
    print("Sending action: ", action)
    sock_game.send(str.encode(action))  # send action to env
    output = recv_socket_data(sock_game)  # get observation from env
    output = json.loads(output)
    steps += 1 """


shopping_list = game_state['observation']['players'][playernumber]['shopping_list']
shopping_quant = game_state['observation']['players'][playernumber]['list_quant']
player = preliminary_path.PathPlanner()
print("Created Preliminary Path")
itemcount = len(shopping_list)

if itemcount > 6:
    print("More than 6, get a cart")
    path = player.grab_cart_or_basket(game_state, kind="cart")
    print("got the path to cart")
    print("Path: ", path)
    state = followplan.ExecutePlanToItem(path, sock_game, playernumber)
    while state != "ERROR" and state != "SUCCESS":
        # make a new plan, this one failed
        state = followplan.ExecutePlanToItem(path, sock_game, playernumber)

    has_cart = True
else:
    print("less than 6, get a basket")
    path = player.grab_cart_or_basket(game_state, kind="basket")
    print("got the path to basket")
    print("Path: ", path)
    state = followplan.ExecutePlanToItem(path, sock_game, playernumber)
    while state == "ERROR" and state != "SUCCESS":
        # keep looping through the plan
        state = followplan.ExecutePlanToItem(path, sock_game, playernumber)

    has_cart = False
## end of the "do once" section

# get all the items on the shopping list
while len(shopping_list) > 0:
    print("Shopping list: ", shopping_list)
    print("Shopping quant: ", shopping_quant)
    sock_game.send(str.encode("0 NOP"))
    print("sent NOP to environment")
    output = recv_socket_data(sock_game)
    print("got output from environment")
    game_state = json.loads(output) # get new state
    print("about to get the next shopping item")
    nextitem = followplan.get_next_shopping_item(game_state, playernumber, shopping_list, shopping_quant)
    print("got the next shopping item")
    nextitemPos = target_locations[nextitem[0]]
    path = player.get_path(game_state, nextitemPos, has_cart)
    # now take the path and excute it
    state = followplan.ExecutePlanToItem(path, sock_game, playernumber)
    if state == "ERROR":
        print("Error")
        # make a new plan, this one failed
    else:
        print("Got the item")
        # make a new plan, but first remove the item we already got from the shopping list
        shopping_list.remove(nextitem)
        shopping_quant.pop(0)
    
path = player.checkout(game_state)
followplan.ExecutePlanToItem(path, sock_game)


sock_game.close()
     
