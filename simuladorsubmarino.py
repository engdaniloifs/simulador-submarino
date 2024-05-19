import numpy as np
import matplotlib.pyplot as plt
import curses  # Importa a biblioteca para capturar a entrada do teclado
import time


#submarine and environmental specifications
x = 0
z = 0
syringemass = 0
waterdensity = 1000 #kg/m^3
submarinevolume = 0.0032 # m^3
submarinemass = 3.185 # kg
gravity = 10 # m/s^2
velocityinitial = 0
syringestep = 0.03/18



class Robot:
    def __init__(self, x, z, syringemass, submarinemass, submarinevolume, velocity,):
        self.x = x
        self.z = z
        self.syringemass = syringemass
        self.mass = submarinemass
        self.volume = submarinevolume
        self.velocity = velocity
        
        
    def move_forward(self):
        self.x += 0.25
        
    def move_backward(self):
        self.x -= 0.25 

    def move_float(self, syringestep):
        self.syringemass -= syringestep
        if self.syringemass <= 0: 
            self.syringemass = 0

    def move_sink(self, syringestep):
        self.syringemass += syringestep
        if self.syringemass >= 0.030: 
            self.syringemass = 0.030

    def move_depth(self, waterdensity,time_initial, gravity):
        acceleration = gravity*((waterdensity*self.volume)/(self.mass + self.syringemass)  -   1 )
        
        time_end = time.time()
        deltatime = time_end - time_initial

        velocity = self.velocity + acceleration*deltatime
        
        self.z = self.z + velocity*deltatime + (acceleration*deltatime*deltatime)/2
        self.velocity = velocity

        print(self.z)
        

        
        if self.z >= 0:
            self.z = 0
            self.velocity = 0
        elif self.z <= -3:
            self.z = -3
            self.velocity = 0    




class Environment:
    def __init__(self, width, depth):
        self.width = width
        self.depth = depth
        self.grid = np.zeros((depth, width))  # Inicializa o grid com todas as células vazias
        self.fig = plt.figure()# Criar a figura e os eixos uma vez
        
        self.ax1 = self.fig.add_subplot(211)
        self.ax1.set_position([0.2, 0.35, 0.6, 0.6])
        self.ax2 = self.fig.add_subplot(212, aspect=0.03, adjustable='box')
        self.ax2.set_position([0.1, 0.1, 0.8, 0.1])

    def add_obstacle(self, x, z):
        self.grid[z][x] = 1  # Define a célula como obstáculo

    def plot_environment(self, robot):
        self.ax1.clear()  # Limpa o eixo antes de plotar novamente
        
        self.ax1.imshow(self.grid, cmap='binary')  # Plota o grid
        self.ax1.scatter(robot.x, robot.z, color='red', marker='o', label='Robot')  # Plota a posição do robô
        self.ax1.legend()
        self.ax1.set_title('Ambiente 2D Grid-based')
        self.ax1.set_xlabel('Posição X')
        self.ax1.set_ylabel('Posição Z')  # Altera o rótulo do eixo y para Z
        self.ax1.set_ylim(-3, 0)  
        self.ax1.set_xlim(0, 3)
        self.ax1.set_xticks([0, 0.5, 1,  1.5, 2, 2.5, 3], 
                            ['0', '0.5', '1',  '1.5', '2', '2.5', '3'])
        self.ax1.grid(True)

        #plt.subplot(1, 2, 2)
        self.ax2.clear()
        self.ax2.plot([0, robot.syringemass], [0, 0], 'k-', lw=5)  # Desenha uma linha horizontal

        #self.ax2.set_ylim(-0.1, 0.1)  # Define os limites do eixo y para a linha
        self.ax2.set_title('Seringa')
        self.ax2.set_xlabel('Massa de água (kg)')
        self.ax2.set_xlim(0, 0.030)
        self.ax2.set_xticks([0, 0.005, 0.010,  0.015, 0.020, 0.025, 0.030], 
                            ['0', '0.005', '0.010',  '0.015', '0.020', '0.025', '0.030'])
        # Removendo as marcações do eixo vertical
        self.ax2.set_yticks([])
        
        #self.fig.tight_layout()
        # Ajustando o layout para evitar sobreposição
        #self.ax2.set_tight_layout()

        plt.show(block=False)
        plt.pause(0.01)  # Pausa para atualizar a interface gráfica

# Função para capturar a entrada do teclado
def get_key(stdscr):
    stdscr.nodelay(True)  # Configura para não bloquear
    return stdscr.getch()


# Criando o ambiente
env_width = 3
env_depth = 3
environment = Environment(env_width, env_depth)


# Adicionando obstáculos ao ambiente (opcional)
#environment.add_obstacle(3, 5)
#environment.add_obstacle(7, 8)

# Criando o robô na posição inicial (0, 0)
robot = Robot(x, z, syringemass, submarinemass, submarinevolume, velocityinitial)

# Visualizar o ambiente inicial


# Mantém o programa rodando até que o usuário pressione 'q'

while True:
    time_initial = time.time()
    environment.plot_environment(robot)
    
    key = curses.wrapper(get_key)
   
    if key == ord('w'):
        robot.move_float(syringestep)
    elif key == ord('s'):
        robot.move_sink(syringestep)
    elif key == ord('a'):
        robot.move_backward()
    elif key == ord('d'):
        robot.move_forward()
    elif key == -1:
        print("Pressione uma tecla")
    elif key == ord('q'):
        print("Obrigado por jogar!")
        break
    
    else:
        print("Tecla inválida! Use as setas do teclado ou 'q' para sair.")
    # Visualizar o ambiente após cada movimento
    robot.move_depth(waterdensity, time_initial, gravity)