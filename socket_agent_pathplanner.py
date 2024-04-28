import json
import random
import socket

from env import SupermarketEnv
from utils import recv_socket_data
import followplan   
import preliminary_path


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
    print("More than 6")
    path = player.grab_cart_or_basket(game_state, kind="cart")
    print("got the path to cart")
    state, path = followplan.ExecutePlanToItem(path, sock_game, playernumber)
    has_cart = True
else:
    print("less than 6")
    path = player.grab_cart_or_basket(game_state, kind="basket")
    print("got the path to basket")
    state, path = followplan.ExecutePlanToItem(path, sock_game, playernumber)
    has_cart = False
## end of the "do once" section

# get all the items on the shopping list
while len(shopping_list) > 0:
    print("Shopping list: ", shopping_list)
    print("Shopping quant: ", shopping_quant)
    nextitem = followplan.get_next_shopping_item(state)
    path = player.get_path(game_state, nextitem, has_cart)
    # now take the path and excute it
    state, path = followplan.ExecutePlanToItem(path, sock_game, playernumber)
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
     
