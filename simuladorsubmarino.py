import numpy as np
import matplotlib.pyplot as plt
import curses  # Importa a biblioteca para capturar a entrada do teclado
import time


#submarine and environmental specifications






class Robot:
    def __init__(self, x, z, syringemass, submarinemass, submarinevolume):
        self.x = x
        self.z = z
        self.syringemass = syringemass
        self.mass = submarinemass
        self.volume = submarinevolume
        self.velocity = 0
        self.acceleration = 0
        
        
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
        self.acceleration = gravity*((waterdensity*self.volume)/(self.mass + self.syringemass)  -   1 )
        
        time_end = time.time()
        deltatime = time_end - time_initial

        velocity = self.velocity + self.acceleration*deltatime
        
        self.z = self.z + velocity*deltatime + (self.acceleration*deltatime*deltatime)/2
        self.velocity = velocity

        
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
        self.fig = plt.figure(figsize=(10, 6))# Criar a figura e os eixos uma vez
        
        self.ax1 = self.fig.add_subplot(221)
        self.ax1.set_position([0.05, 0.35, 0.6, 0.6])
        self.ax2 = self.fig.add_subplot(223, aspect=0.03, adjustable='box')
        self.ax2.set_position([0.1, 0.1, 0.8, 0.1])
        self.ax3 = self.fig.add_subplot(222)
        self.ax3.set_position([0.6, 0.6, 0.35, 0.15])

    def add_obstacle(self, x, z):
        self.grid[z][x] = 1  # Define a célula como obstáculo

    def plot_environment(self, robot):
        self.ax1.clear()  # Limpa o eixo antes de plotar novamente

        # Plota o fundo primeiro 

        # Plota o grid
        self.ax1.imshow(self.grid, cmap='binary',  zorder=0)  

        self.ax1.imshow(aquario_img, extent=[0, 3, -3, 0], aspect='auto', zorder=1) 

        # Define a posição onde a imagem do submarino será desenhada
        img_extent = [robot.x - 0.15, robot.x + 0.15, robot.z - 0.1, robot.z + 0.1]  

        # Plota a imagem do submarino sobre o fundo
        self.ax1.imshow(submarine_img, extent=img_extent, aspect='auto', zorder=2)  # Maior zorder
        

        # Configurações do gráfico
        self.ax1.set_title('Ambiente 2D Grid-based')
        self.ax1.set_xlabel('Position X (m)')
        self.ax1.set_ylabel('Depth(m)')  # Altera o rótulo do eixo Y para Z
        self.ax1.set_ylim(-3, 1)  
        self.ax1.set_xlim(0, 3)

        # Definir os ticks do eixo X
        self.ax1.set_xticks([0, 0.5, 1,  1.5, 2, 2.5, 3], 
                            ['0', '0.5', '1',  '1.5', '2', '2.5', '3'])

        self.ax1.grid(True)

        self.ax2.clear()
        self.ax2.plot([0, robot.syringemass], [0, 0], 'k-', lw=5)  # Desenha uma linha horizontal
        self.ax2.set_title('Syringe')
        self.ax2.set_xlabel('Water mass (kg)')
        self.ax2.set_xlim(0, 0.030)
        self.ax2.set_xticks([0, 0.005, 0.010,  0.015, 0.020, 0.025, 0.030], 
                            ['0', '0.005', '0.010',  '0.015', '0.020', '0.025', '0.030'])
        # Removendo as marcações do eixo vertical
        self.ax2.set_yticks([])
        
        data = np.array([['Depth (m)', '{:.4f}'.format(robot.z)],
                         ['Velocity (m/s)', '{:.4f}'.format(robot.velocity)],
                         ['Acceleration (m/s^2)', '{:.4f}'.format(robot.acceleration)]])
        self.ax3.clear()
        table = self.ax3.table(cellText=data, colLabels=['Column1', 'Column2'], loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 1)
        self.ax3.axis('tight')
        self.ax3.axis('off')

       # plt.show(block=False)
        plt.pause(0.01)  # Pausa para atualizar a interface gráfica

# Função para capturar a entrada do teclado
def get_key(stdscr):
    stdscr.nodelay(True)  # Configura para não bloquear
    return stdscr.getch()


# Criando o ambiente
env_width = 3
env_depth = 3
environment = Environment(env_width, env_depth)

x = 2
z = -2
syringemass = 0
waterdensity = 1000 #kg/m^3
submarinevolume = 0.0032 # m^3
submarinemass = 3.185 # kg
gravity = 10 # m/s^2
syringestep = 0.03/18

submarine_img = plt.imread("D:\\Danilo\\projetos\\github\\simulador-submarino\\submarine.png")

aquario_img = plt.imread("D:\\Danilo\\projetos\\github\\simulador-submarino\\aquario.png")
# Adicionando obstáculos ao ambiente (opcional)
#environment.add_obstacle(3, 5)
#environment.add_obstacle(7, 8)

# Criando o robô na posição inicial (0, 0)
robot = Robot(x, z, syringemass, submarinemass, submarinevolume)


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