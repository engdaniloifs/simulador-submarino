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
        self.velocity_x = 0  # Velocidade no eixo X
        self.velocity_z = 0  # Velocidade no eixo Z
        self.acceleration = 0
        self.time_last_update = time.time()  # Para controle do tempo
        
    def move_forward(self):
        """Acelera suavemente para frente"""
        self.velocity_x += 0.05  # Aumenta a velocidade gradualmente
        
    def move_backward(self):
        """Acelera suavemente para trás"""
        self.velocity_x -= 0.05  # Aumenta a velocidade gradualmente no sentido oposto  

    def move_float(self, syringestep):
        """Reduz a massa da seringa para subir"""
        self.syringemass = max(0, self.syringemass - syringestep)

    def move_sink(self, syringestep):
        """Aumenta a massa da seringa para descer"""
        self.syringemass = min(0.030, self.syringemass + syringestep)

    def update_motion(self, waterdensity, gravity):
        """Atualiza a posição X e Z do robô suavemente"""
        time_now = time.time()
        dt = time_now - self.time_last_update  # Tempo decorrido
        self.time_last_update = time_now  # Atualiza o tempo

        # Atualiza posição X baseado na velocidade
        self.x += self.velocity_x * dt  
        self.velocity_x *= 0.95  # Simula resistência do meio (desaceleração)

        # Calcula aceleração na profundidade com base no empuxo
        self.acceleration = gravity * ((waterdensity * self.volume) / (self.mass + self.syringemass) - 1)

        # Atualiza velocidade e posição Z
        self.velocity_z += self.acceleration * dt
        self.z += self.velocity_z * dt + 0.5 * self.acceleration * dt**2

        # Limites de profundidade
        self.z = max(-3, min(0, self.z))
        self.x = max(0, min(3, self.x))

        if self.z == 0 or self.z == -3:
            self.velocity_z = 0
        if self.x == 0 or self.x == 3:
            self.velocity_x = 0




class Environment:
    def __init__(self, width, depth, aquario_img, submarine_img):
         # Inicializa o grid com todas as células vazias
        self.fig = plt.figure(figsize=(10, 6))  

        # Adicionando subplots manualmente com posições definidas em [left, bottom, width, height]
        self.ax1 = self.fig.add_axes([0.05, 0.35, 0.55, 0.6])  # Superior esquerdo
        self.ax3 = self.fig.add_axes([0.65, 0.60, 0.30, 0.25])  # Superior direito
        self.ax2 = self.fig.add_axes([0.10, 0.10, 0.80, 0.15])  # Inferior, ocupando a largura

        # Configuração inicial do ambiente
        self.width = width
        self.depth = depth
        

        self.ax1.set_title('Ambiente 2D Grid-based')
        self.ax1.set_xlabel('Position X (m)')
        self.ax1.set_ylabel('Depth (m)')
        self.ax1.set_ylim(-3, 1)
        self.ax1.set_xlim(0, 3)
        self.ax1.set_xticks(np.arange(0, 3.5, 0.5))
        self.ax1.grid(True)

        # Plota o fundo e salva a referência da imagem
       
        self.aquario_plot = self.ax1.imshow(aquario_img, extent=[0, 3, -3, 0], aspect='auto', zorder=0)
        self.submarine_plot = self.ax1.imshow(submarine_img, extent=[0, 0, 0, 0], aspect='auto', zorder=1)  # Inicializa com extensão vazia

        # Configuração do segundo gráfico (Syringe)
        self.ax2.set_title('Syringe')
        self.ax2.set_xlabel('Water mass (kg)')
        self.ax2.set_xlim(0, 0.030)
        self.ax2.set_xticks(np.linspace(0, 0.030, 7))
        self.ax2.set_yticks([])
        self.syringe_line, = self.ax2.plot([], [], 'k-', lw=5)  # Inicializa a linha vazia

        # Configuração do terceiro gráfico (tabela)
        self.ax3.axis('tight')
        self.ax3.axis('off')
        self.table = None  # Inicializa a tabela

    def update_environment(self, robot):
        # Atualiza a posição da imagem do submarino
        img_extent = [robot.x - 0.15, robot.x + 0.15, robot.z - 0.1, robot.z + 0.1]
        self.submarine_plot.set_extent(img_extent)

        # Atualiza o gráfico da seringa
        self.syringe_line.set_data([0, robot.syringemass], [0, 0])

        # Atualiza a tabela de informações
        data = np.array([
            ['Depth (m)', '{:.4f}'.format(robot.z)],
            ['Velocity (m/s)', '{:.4f}'.format(robot.velocity_z)],
            ['Acceleration (m/s^2)', '{:.4f}'.format(robot.acceleration)]
        ])

        if self.table:
            self.table.remove()  # Remove a tabela antiga
        self.table = self.ax3.table(cellText=data, loc='center', cellLoc='center')
        self.table.auto_set_font_size(False)
        self.table.set_fontsize(10)
        self.table.scale(1, 1)

        # Atualiza a interface gráfica sem limpar tudo
        plt.pause(0.0001)

# Função para capturar a entrada do teclado
def get_key(stdscr):
    stdscr.nodelay(True)  # Configura para não bloquear
    return stdscr.getch()


# Criando o ambiente
env_width = 3
env_depth = 3
submarine_img = plt.imread("D:\\Danilo\\projetos\\github\\simulador-submarino\\submarine.png")

aquario_img = plt.imread("D:\\Danilo\\projetos\\github\\simulador-submarino\\aquario.png")

environment = Environment(env_width, env_depth,aquario_img,submarine_img)

x = 2
z = -2
syringemass = 0
waterdensity = 1000 #kg/m^3
submarinevolume = 0.0032 # m^3
submarinemass = 3.185 # kg
gravity = 10 # m/s^2
syringestep = 0.03/18


# Adicionando obstáculos ao ambiente (opcional)
#environment.add_obstacle(3, 5)
#environment.add_obstacle(7, 8)

# Criando o robô na posição inicial (0, 0)
robot = Robot(x, z, syringemass, submarinemass, submarinevolume)


while True:
    time_initial = time.time()
    environment.update_environment(robot)
    
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
    robot.update_motion(waterdensity, gravity)