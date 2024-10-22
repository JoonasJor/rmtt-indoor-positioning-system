import pygame

window_size_x = 1400
window_size_y = 1050
beacon_to_move = 1
target_fps = 60 

def initialize_pygame():
    pygame.init()
    font = pygame.font.SysFont(None, 20)
    screen = pygame.display.set_mode([window_size_x, window_size_y], pygame.SCALED | pygame.FULLSCREEN)
    background_image = pygame.image.load("images/2800x2100.png")
    background_image = pygame.transform.scale(background_image, (window_size_x, window_size_y))
    clock = pygame.time.Clock()
    return font, screen, background_image, clock, target_fps