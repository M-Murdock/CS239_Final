import pygame
import config
# import random
from enums.cart_state import CartState
from helper import obj_collision, can_interact_default
from render_game import render_text
from objects import InteractiveObject


class Counter(InteractiveObject):
    def can_interact(self, player):
        return can_interact_default(self, player)

    def __init__(self, x_position, y_position, image, food_image, string_type):
        super().__init__(num_stages=2)
        self.position = [x_position, y_position]
        self.image = image
        self.food_image = food_image
        self.string_type = string_type
        self.width = 0.5
        self.height = 2

    def collision(self, x_position, y_position):
        return obj_collision(self, x_position, y_position, x_margin=0.55, y_margin=0.55)

    def render(self, screen, camera):
        screen.blit(self.image, ((self.position[0] * config.SCALE) - (camera.position[0] * config.SCALE),
                                 (self.position[1] * config.SCALE) - (camera.position[1] * config.SCALE)))

    def interact(self, game, player):
        if self.interactive_stage == 0:
            self.interaction_message = "Hello! Would you like to buy " + self.string_type + "?"
        elif self.interactive_stage == 1:
            if player.curr_cart is None and player.holding_food is None:
                player.hold_food(self.string_type, self.food_image)
                self.interaction_message = "You picked up your order."
            elif player.holding_food is not None:
                self.interaction_message = "Let go of the food you're holding to pick up food here!"
            else:
                self.interaction_message = "Let go of your cart to pick up food here!"
