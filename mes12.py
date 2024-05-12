import numpy as np
import matplotlib.pyplot as plt
import curses  # Importa a biblioteca para capturar a entrada do teclado

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.direction = 'up'  # Direção inicial do robô

    def move_forward(self):
        if self.direction == 'up':
            self.y += 1
        elif self.direction == 'down':
            self.y -= 1
        elif self.direction == 'left':
            self.x -= 1
        elif self.direction == 'right':
            self.x += 1

    def move_backward(self):
        if self.direction == 'up':
            self.y -= 1
        elif self.direction == 'down':
            self.y += 1
        elif self.direction == 'left':
            self.x += 1
        elif self.direction == 'right':
            self.x -= 1

    def turn_left(self):
        if self.direction == 'up':
            self.direction = 'left'
        elif self.direction == 'down':
            self.direction = 'right'
        elif self.direction == 'left':
            self.direction = 'down'
        elif self.direction == 'right':
            self.direction = 'up'

    def turn_right(self):
        if self.direction == 'up':
            self.direction = 'right'
        elif self.direction == 'down':
            self.direction = 'left'
        elif self.direction == 'left':
            self.direction = 'up'
        elif self.direction == 'right':
            self.direction = 'down'

class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))  # Inicializa o grid com todas as células vazias
        self.fig, self.ax = plt.subplots()  # Criar a figura e os eixos uma vez

    def add_obstacle(self, x, y):
        self.grid[y][x] = 1  # Define a célula como obstáculo

    def plot_environment(self, robot):
        self.ax.clear()  # Limpa o eixo antes de plotar novamente
        self.ax.imshow(self.grid, cmap='binary')  # Plota o grid
        self.ax.scatter(robot.x, robot.y, color='red', marker='o', label='Robot')  # Plota a posição do robô
        self.ax.legend()
        self.ax.set_title('Ambiente 2D Grid-based')
        self.ax.set_xlabel('Posição X')
        self.ax.set_ylabel('Posição Y')
        self.ax.grid(True)
        plt.show(block=False)
        plt.pause(1)  # Pausa para atualizar a interface gráfica

# Função para capturar a entrada do teclado
def get_key(stdscr):
    key = stdscr.getch()
    return key

# Criando o ambiente
env_width = 10
env_height = 10
environment = Environment(env_width, env_height)


# Adicionando obstáculos ao ambiente (opcional)
environment.add_obstacle(3, 5)
environment.add_obstacle(7, 8)

# Criando o robô na posição inicial (0, 0)
robot = Robot(0, 0)

# Visualizar o ambiente inicial
environment.plot_environment(robot)

# Mantém o programa rodando até que o usuário pressione 'q'

while True:
    key = curses.wrapper(get_key)
   
    if key == ord('w'):
        robot.move_forward()
    elif key == ord('s'):
        robot.move_backward()
    elif key == ord('a'):
        robot.turn_left()
    elif key == ord('d'):
        robot.turn_right()
    elif key == ord('q'):
        print("Obrigado por jogar!")
        break
    
    else:
        print("Tecla inválida! Use as setas do teclado ou 'q' para sair.")

    # Visualizar o ambiente após cada movimento
    environment.plot_environment(robot)