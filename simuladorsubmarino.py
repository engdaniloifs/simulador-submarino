import numpy as np
import matplotlib.pyplot as plt
import curses  # Importa a biblioteca para capturar a entrada do teclado

class Robot:
    def __init__(self, x, z, syringemass):
        self.x = x
        self.z = 0
        self.syringemass = 0
        
    def move_forward(self):
        self.x += 1
        
    def move_backward(self):
        self.x -= 1 

    def move_float(self):
        self.syringemass -= 15
        if self.syringemass <= 0: 
            self.syringemass = 0

    def move_sink(self):
        self.syringemass += 15
        if self.syringemass >= 30: 
            self.syringemass = 30

    def move_depth(self):
        if self.syringemass == 30:
            self.z -= 1
        elif self.syringemass == 0:
            self.z += 1
        if self.z >= 0:
            self.z = 0
        elif self.z <= -30:
            self.z = -30    




class Environment:
    def __init__(self, width, depth):
        self.width = width
        self.depth = depth
        self.grid = np.zeros((depth, width))  # Inicializa o grid com todas as células vazias
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2) # Criar a figura e os eixos uma vez

    def add_obstacle(self, x, z):
        self.grid[z][x] = 1  # Define a célula como obstáculo

    def plot_environment(self, robot):
        self.ax1.clear()  # Limpa o eixo antes de plotar novamente
        self.ax1.imshow(self.grid, cmap='binary')  # Plota o grid
        self.ax1.scatter(robot.x, -robot.z, color='red', marker='o', label='Robot')  # Plota a posição do robô
        self.ax1.legend()
        self.ax1.set_title('Ambiente 2D Grid-based')
        self.ax1.set_xlabel('Posição X')
        self.ax1.set_ylabel('Posição Z')  # Altera o rótulo do eixo y para Z
        self.ax1.grid(True)

        plt.subplot(1, 2, 2)
        self.ax2.clear()
        plt.plot([0, robot.syringemass], [0, 0], 'k-', lw=5)  # Desenha uma linha horizontal

        plt.ylim(-0.1, 0.1)  # Define os limites do eixo y para a linha
        plt.title('Seringa')

        plt.xlim(0, 30)
        plt.xticks([0, 15, 30], ['0', '15', '30'])
        # Removendo as marcações do eixo vertical
        plt.yticks([])

        # Ajustando o layout para evitar sobreposição
        plt.tight_layout()

        plt.show(block=False)
        plt.pause(1)  # Pausa para atualizar a interface gráfica

# Função para capturar a entrada do teclado
def get_key(stdscr):
    key = stdscr.getch()
    return key

# Criando o ambiente
env_width = 30
env_depth = 30
environment = Environment(env_width, env_depth)


# Adicionando obstáculos ao ambiente (opcional)
environment.add_obstacle(3, 5)
environment.add_obstacle(7, 8)

# Criando o robô na posição inicial (0, 0)
robot = Robot(0, 0, 0)

# Visualizar o ambiente inicial
environment.plot_environment(robot)

# Mantém o programa rodando até que o usuário pressione 'q'

while True:
    key = curses.wrapper(get_key)
   
    if key == ord('w'):
        robot.move_float()
    elif key == ord('s'):
        robot.move_sink()
    elif key == ord('a'):
        robot.move_backward()
    elif key == ord('d'):
        robot.move_forward()
    elif key == ord('q'):
        print("Obrigado por jogar!")
        break
    
    else:
        print("Tecla inválida! Use as setas do teclado ou 'q' para sair.")

    robot.move_depth()
    # Visualizar o ambiente após cada movimento
    environment.plot_environment(robot)