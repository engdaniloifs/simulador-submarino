import numpy as np
import matplotlib.pyplot as plt
import time


#submarine and environmental specifications

class Robot:
    def __init__(self, x, z, syringemass, submarinemass, submarinevolume):
        self.x = x
        self.z = z
        self.syringemass = syringemass
        self.mass = submarinemass
        self.volume = submarinevolume 
        self.velocity_z = 0  
        self.acceleration = 0
        self.time_last_update = time.time()  
        self.syringe_velocity = 0.015  # 30g por segundo
        self.syringe_target = self.syringemass  # Inicializa com o valor atual
        self.setpointz = 0
        self.kp = 0.020 
        self.ki = 0.0000
        self.kd = 0.22
        
        self.error_integral = 0
        self.prev_error = 0
        #self.kd = [0.05,0.06, 0.07, 0.08, 0.09]
        self.syringe_neutral = 0.015





    def update_motion(self, waterdensity, gravity,i):
        time_now = time.time()
        dt = time_now - self.time_last_update  
        self.time_last_update = time_now  

        # Atualiza posição X baseado na velocidade


        error = -(self.setpointz - self.z)

        # Atualiza o erro acumulado (integral)
        self.error_integral += error * dt  

        # Calcula a derivada do erro
        error_derivative = (error - self.prev_error) / dt  

        # Controle PID (P + I + D)
        self.syringe_target = self.syringe_neutral + (error * self.kp) + (self.error_integral * self.ki) + (error_derivative * self.kd)

        # Garante que o valor fique dentro dos limites
        if abs(self.syringemass - self.syringe_target) > 0.0083:
            if self.syringemass > self.syringe_target:
                self.syringe_target = self.syringemass - 0.0083
            else:
                self.syringe_target = self.syringemass + 0.0083

        if abs(self.syringemass - self.syringe_target) < 0.00083:
            self.syringe_target = self.syringemass
        testeservo = self.syringe_target*6000

        self.syringe_target = int(testeservo)/6000

        self.syringe_target = max(0, min(self.syringe_target, 0.03))

        # Atualiza o erro anterior para a próxima iteração
        self.prev_error = error  

        


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

        if self.z == 0 or self.z == -3:
            self.velocity_z = 0



class Environment:
    def __init__(self, width, depth, aquario_img, submarine_img, robot):
        
        self.fig = plt.figure(figsize=(12, 6))
        
        self.ax1 = self.fig.add_axes([0.05, 0.55, 0.4, 0.4])  # Submarino
        self.ax4 = self.fig.add_axes([0.55, 0.55, 0.4, 0.4])  # Gráfico de profundidade
        self.ax2 = self.fig.add_axes([0.05, 0.35, 0.9, 0.1])  # Seringa
        self.ax3 = self.fig.add_axes([0.4, 0.05, 0.25, 0.3])  # Tabela
        
        
        # Configuração do ambiente 2D
        self.width = width
        self.depth = depth
        
        self.ax1.set_title('Ambiente 2D Grid-based')
        self.ax1.set_xlabel('Position X (m)')
        self.ax1.set_ylabel('Depth (m)')
        self.ax1.set_ylim(-3, 1)
        self.ax1.set_xlim(0, 3)
        self.ax1.set_xticks(np.arange(0, 3.5, 0.5))
        self.ax1.grid(True)
        
        self.aquario_plot = self.ax1.imshow(aquario_img, extent=[0, 3, -3, 0], aspect='auto', zorder=0)
        self.submarine_plot = self.ax1.imshow(submarine_img, extent=[0, 0, 0, 0], aspect='auto', zorder=1)
        
        # Configuração do gráfico da seringa
        self.ax2.set_title('Syringe')
        self.ax2.set_xlabel('Water mass (kg)')
        self.ax2.set_xlim(0, 0.030)
        self.ax2.set_xticks(np.linspace(0, 0.030, 7))
        self.ax2.set_yticks([])
        self.syringe_line, = self.ax2.plot([], [], 'b-', lw=5)
        
        # Configuração da tabela
        self.ax3.axis('tight')
        self.ax3.axis('off')
        
        data = [
            ['Depth (m)', '{:.4f}'.format(robot.z)],
            ['Velocity (m/s)', '{:.4f}'.format(robot.velocity_z)],
            ['Acceleration (m/s²)', '{:.4f}'.format(robot.acceleration)]
        ]
        
        self.table = self.ax3.table(cellText=data, loc='center', cellLoc='center')
        self.table.auto_set_font_size(False)
        self.table.set_fontsize(10)
        self.table.scale(1, 1)
        
        # Configuração do gráfico de profundidade ao longo do tempo
        self.ax4.set_title('Eixo Z ao longo do Tempo')
        self.ax4.set_xlabel('Tempo (s)')
        self.ax4.set_ylabel('Profundidade (m)')
        self.ax4.set_xlim(0, 120)
        self.ax4.set_ylim(-3, 0)
        self.ax4.grid(True)
        
        
        self.setpoint_line, = self.ax4.plot([0, 120], [robot.setpointz, robot.setpointz], 'g--', lw=2, label='Setpoint')

        colors = ['b', 'c', 'm', 'y', 'k']  # Azul, Ciano, Magenta, Amarelo, Preto

        self.kp_lines = []
        self.kp_time_data = [[] for _ in range(5)]  # Inicializa listas separadas para tempo
        self.kp_depth_data = [[] for _ in range(5)]  # Inicializa listas separadas para profundidade
        
        for i, color in enumerate(colors):
            line, = self.ax4.plot(self.kp_time_data[i], self.kp_depth_data[i], color=color, lw=2, label=f'Kp = {robot.kp}')
            self.kp_lines.append(line)
        
        self.ax4.legend()
    
    def update_environment(self, robot, time, i):
        img_extent = [robot.x - 0.15, robot.x + 0.15, robot.z - 0.1, robot.z + 0.1]
        self.submarine_plot.set_extent(img_extent)
        
        self.syringe_line.set_data([0, robot.syringemass], [0, 0])
        
        self.table[(0, 1)].get_text().set_text('{:.4f}'.format(robot.z))
        self.table[(1, 1)].get_text().set_text('{:.4f}'.format(robot.velocity_z))
        self.table[(2, 1)].get_text().set_text('{:.4f}'.format(robot.acceleration))
        
        self.kp_time_data[i].append(time)  # Adiciona o tempo correspondente
        self.kp_depth_data[i].append(robot.z)  # Adiciona a profundidade correspondente
        
        # Atualiza a linha correspondente no gráfico
        self.kp_lines[i].set_data(self.kp_time_data[i], self.kp_depth_data[i])

        
        
        self.ax4.relim()
        self.ax4.autoscale_view()
        
        plt.pause(0.0001)






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
robot.setpointz = -0.5
environment = Environment(env_width, env_depth,aquario_img,submarine_img,robot)

for i in range(5):  # Vai de 0 a 4
    time_initial = time.time()
    robot.z = 0
    
    while True:
        time_graph = time.time() - time_initial
        environment.update_environment(robot,time_graph,i)
        
        # Visualizar o ambiente após cada movimento
        robot.update_motion(waterdensity, gravity,i) 

        time.sleep(0.01)
        if(time_graph >= 120):
            break

    while True:
        print("espere")