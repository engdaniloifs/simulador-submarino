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
        self.velocity_x = 0  
        self.velocity_z = 0  
        self.acceleration = 0
        self.time_last_update = time.time()  
        self.syringe_velocity = 0.015  # 30g por segundo
        self.syringe_target = self.syringemass  # Inicializa com o valor atual
        self.setpointz = 0
        

    def move_forward(self):
        self.velocity_x += 0.05  

    def move_backward(self):
        self.velocity_x -= 0.05  

    def move_float(self):
        """Reduz a massa da seringa em pequenos passos para subir"""
        self.setpointz = self.setpointz + 0.5 
        

    def move_sink(self):
        """Aumenta a massa da seringa em pequenos passos para descer"""
        self.setpointz = self.setpointz - 0.5 


    def update_motion(self, waterdensity, gravity):
        time_now = time.time()
        dt = time_now - self.time_last_update  
        self.time_last_update = time_now  

        # Atualiza posição X baseado na velocidade
        self.x += self.velocity_x * dt  
        self.velocity_x *= 0.95  

        
        self.syringe_target = 0.015 - (self.setpointz - self.z)*0.01
        
        self.syringe_target = max(0, min(self.syringe_target, 0.03))

        
        # Ajusta a massa da seringa suavemente
        if self.syringemass < self.syringe_target:
            self.syringemass = min(self.syringe_target, self.syringemass + self.syringe_velocity * dt)
        elif self.syringemass > self.syringe_target:
            self.syringemass = max(self.syringe_target, self.syringemass - self.syringe_velocity * dt)

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
    def __init__(self, width, depth, aquario_img, submarine_img, robot):
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
        self.syringe_line, = self.ax2.plot([], [], 'b-', lw=5)  # Inicializa a linha vazia

        # Configuração do terceiro gráfico (tabela)
        self.ax3.axis('tight')
        self.ax3.axis('off')
        self.table = None  # Inicializa a tabela

        data = [
            ['Depth (m)', '{:.4f}'.format(robot.z)],
            ['Velocity (m/s)', '{:.4f}'.format(robot.velocity_z)],
            ['Acceleration (m/s²)', '{:.4f}'.format(robot.acceleration)]
        ]
        
        self.table = self.ax3.table(cellText=data, loc='center', cellLoc='center')
        self.table.auto_set_font_size(False)
        self.table.set_fontsize(10)
        self.table.scale(1, 1)
        self.ax3.axis('off')

    def update_environment(self, robot):
        # Atualiza a posição da imagem do submarino
        img_extent = [robot.x - 0.15, robot.x + 0.15, robot.z - 0.1, robot.z + 0.1]
        self.submarine_plot.set_extent(img_extent)

        # Atualiza o gráfico da seringa
        self.syringe_line.set_data([0, robot.syringemass], [0, 0])

        # Atualiza a tabela de informações
        self.table[(0, 1)].get_text().set_text('{:.4f}'.format(robot.z))
        self.table[(1, 1)].get_text().set_text('{:.4f}'.format(robot.velocity_z))
        self.table[(2, 1)].get_text().set_text('{:.4f}'.format(robot.acceleration))

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



x = 1
z = 0
syringemass = 0
waterdensity = 1000 #kg/m^3
submarinevolume = 0.0032 # m^3
submarinemass = 3.185 # kg
gravity = 10 # m/s^2



while True:
    print("Use as teclas WASD para se movimentar")
    print("Digite 'start' para começar a simulação")
    
    user_input = input()  # Agora o programa espera a entrada do usuário
    
    if user_input.lower() == "start":  
        print("Simulação iniciada!")
        break  # Sai do loop quando o usuário digitar "start"

robot = Robot(x, z, syringemass, submarinemass, submarinevolume)
environment = Environment(env_width, env_depth,aquario_img,submarine_img,robot)


while True:
    time_initial = time.time()
    environment.update_environment(robot)
    
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
    
    # Visualizar o ambiente após cada movimento
    robot.update_motion(waterdensity, gravity)  