import sys
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import streamlit as st
import streamlit.components.v1 as components

class RoboticArm :
    def __init__ (self,link_lengths):
        self.lengths = link_lengths
        self.num_links = len(link_lengths)


    def forward_kinematics(self, angles):
        x,y = 0.0,0.0
        joint_positions = [(x,y)]
        current_angle = 0.0

        for i in range(self.num_links):
            current_angle += angles[i]

            x += self.lengths[i] * np.cos(current_angle)
            y += self.lengths[i] * np.sin(current_angle)

            joint_positions.append((x,y))
        
        return joint_positions
    
class GeneticAlgorithm:
    def __init__(self,arm,target,population_size=100,mutation_rate=.2):
        self.arm=arm
        self.target=target
        self.population_size=population_size
        self.mutation_rate=mutation_rate
        self.num_genes=arm.num_links
        self.angle_min=-np.pi
        self.angle_max=np.pi

    def create_population(self):
        return np.random.uniform(self.angle_min,self.angle_max,(self.population_size,self.num_genes))
    
    def selection(self, population, fitnesses):

        tournament_indices = np.random.choice(len(population),size=3,replace=False)

        best_index = tournament_indices[np.argmax(fitnesses[tournament_indices])]
        return population[best_index]
    
    def crossover(self, parent1,parent2):
        child =np.copy(parent1)
        for i in range (self.num_genes):
            if np.random.rand() >.5:
                child[i]=parent2[i]
        return child
    
    def mutate(self,child):
        for i in range(self.num_genes):
            if np.random.rand() < self.mutation_rate:
                child[i] += np.random.normal(0,1.5)
                child[i]=np.clip(child[i],self.angle_min,self.angle_max)
        return child
    
    # 1. ضفنا progress_bar و status_text كمتغيرات اختيارية (علشان لو شغلنا من الكوماند لاين متعملش مشكلة)
    def run(self, generations=1000, progress_bar=None, status_text=None):
        population = self.create_population()
        best_solution = None
        best_fitness = -1

        history = [] 

        for gen in range(generations+1):
            fitnesses  = np.array([fitness(ind, self.arm, self.target) for ind in population])
            current_best_index = np.argmax(fitnesses)
            if fitnesses[current_best_index] > best_fitness:
                best_fitness = fitnesses[current_best_index]
                best_solution = population[current_best_index]

            history.append(best_solution.copy())

            new_population = []

            new_population.append(best_solution)

            while len(new_population) < self.population_size:
                p1 = self.selection(population, fitnesses)
                p2 = self.selection(population, fitnesses)
                child = self.crossover(p1,p2)
                child = self.mutate(child)
                new_population.append(child)
            
            population = np.array(new_population)

            if gen % 10 == 0:
                msg = f"⏳ Generation {gen} | Best Fitness: {best_fitness:.4f}"
                print(msg)
                if status_text is not None: 
                    status_text.text(msg) 
            
            if progress_bar is not None:
                progress_bar.progress((gen + 1) / (generations + 1))

        return best_solution, history
        

###### -------- Fitness Function -------- ###### 
def fitness(angles,arm,target):

    joints = arm.forward_kinematics(angles)
    end_effector = joints[-1]

    distance = np.sqrt(np.array((end_effector[0]-target[0])**2 + (end_effector[1]-target[1])**2))

    e = 1e-6

    fitness = 1.0 / (distance + e)
    return fitness

###### -------- Main Execution -------- ####


def run_local():
    while True:
        print("\n" + "="*40)
        print("🤖 Robotic Arm Setup (Local Mode)")
        print("="*40)

        while True:
            lengths_str = input("👉 Enter link lengths separated by commas (e.g., 2.0, 1.5, 1.0): ")
            try:
                arm_lengths = [float(x.strip()) for x in lengths_str.split(",")]
                if len(arm_lengths) < 1:
                    raise ValueError
                break
            except ValueError:
                print("❌ Invalid input. Please enter numbers separated by commas.")

        while True:
            try:
                target_x = float(input("👉 Enter Target X coordinate (e.g., 3.0): "))
                target_y = float(input("👉 Enter Target Y coordinate (e.g., 4.0): "))
                target_point = (target_x, target_y)
                break
            except ValueError:
                print("❌ Invalid input. Please enter valid numbers.")

        print("\n--- Genetic Algorithm Settings ---")
        
        while True:
            try:
                pop_size = int(input("👉 Enter Population Size (e.g., 200): "))
                if pop_size <= 0:
                    print("❌ Size must be greater than 0.")
                    continue
                break
            except ValueError:
                print("❌ Invalid input. Please enter an integer number.")

        while True:
            try:
                mut_rate = float(input("👉 Enter Mutation Rate (between 0.0 and 1.0, e.g., 0.3): "))
                if not (0.0 <= mut_rate <= 1.0):
                    print("❌ Rate must be between 0.0 and 1.0.")
                    continue
                break
            except ValueError:
                print("❌ Invalid input. Please enter a decimal number.")

        while True:
            try:
                max_gen = int(input("👉 Enter Max Generations (e.g., 100): "))
                if max_gen <= 0:
                    print("❌ Generations must be greater than 0.")
                    continue
                break
            except ValueError:
                print("❌ Invalid input. Please enter an integer number.")

        arm = RoboticArm(arm_lengths)
        
        print("\nTraining Genetic Algorithm... Please wait.")
        ga = GeneticAlgorithm(arm=arm, target=target_point, population_size=pop_size, mutation_rate=mut_rate)
        best_angles, history = ga.run(generations=max_gen) 

        final_joints = arm.forward_kinematics(best_angles)
        final_end_effector = final_joints[-1]
        error_distance = np.sqrt((final_end_effector[0]-target_point[0])**2 + (final_end_effector[1]-target_point[1])**2)
        
        print("\n" + "="*30)
        print("📊 Final Results:")
        print(f"Target Point  : {target_point}")
        print(f"Arm Reached   : ({final_end_effector[0]:.4f}, {final_end_effector[1]:.4f})")
        print(f"Error Distance: {error_distance:.4f} units")
        if error_distance < 0.05:
            print("Accuracy      : EXCELLENT (Hit the target)")
        else:
            print("Accuracy      : GOOD (Needs more generations or population)")
        print("="*30 + "\n")

        max_reach = sum(arm_lengths)
        plot_limit = max(max_reach + 1.0, max(abs(target_point[0]), abs(target_point[1])) + 1.0)

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim(-plot_limit, plot_limit)
        ax.set_ylim(-plot_limit, plot_limit)
        ax.set_title(f'Robotic Arm GA Training\nPop: {pop_size} | Mut: {mut_rate} | Gen: {max_gen}', fontsize=14)
        ax.grid(True, linestyle='--', alpha=0.6)
        
        ax.plot(target_point[0], target_point[1], 'r*', markersize=15, label='Target')
        ax.legend()

        line, = ax.plot([], [], 'o-', lw=4, markersize=8, color='#38bdf8')
        generation_text = ax.text(-plot_limit + 0.5, plot_limit - 1.0, '', fontsize=12, color='black')

        def update(frame):
            current_angles = history[frame]
            joints = arm.forward_kinematics(current_angles)
            
            x_coords = [j[0] for j in joints]
            y_coords = [j[1] for j in joints]
            
            line.set_data(x_coords, y_coords)
            generation_text.set_text(f'Generation: {frame}')
            return line, generation_text

        print("Rendering Animation... (Close the plot window to continue)")
        ani = animation.FuncAnimation(fig, update, frames=len(history), interval=100, blit=True, repeat=False)
        
        plt.show()

        print("\n" + "-"*40)
        retry = input("🔄 Do you want to try again with different inputs? (y/n): ").strip().lower()
        if retry != 'y':
            print("Exiting Local Simulation. Goodbye! 👋")
            break



def run_streamlit():
    st.set_page_config(page_title="Robotic Arm GA", layout="wide")
    st.title("🤖 Robotic Arm Kinematics with Genetic Algorithm")
    st.write("Interactive dashboard to train the robotic arm to reach the target.")

    st.sidebar.header("⚙️ Algorithm Settings")
    pop_size = st.sidebar.slider("Population Size", 10, 500, 150)
    mut_rate = st.sidebar.slider("Mutation Rate", 0.0, 1.0, 0.2)
    max_gen = st.sidebar.slider("Max Generations", 50, 500, 200)
    
    st.sidebar.header("📏 Arm Configuration")
    lengths_str = st.sidebar.text_input("Link Lengths (comma-separated)", "2.0, 1.5, 1.0")
    
    try:
        link_lengths = [float(x.strip()) for x in lengths_str.split(",")]
    except ValueError:
        st.sidebar.error("Invalid input format. Using default [2.0, 1.5, 1.0].")
        link_lengths = [2.0, 1.5, 1.0]

    st.sidebar.header("🎯 Target Coordinates")
    target_x = st.sidebar.number_input("Target X", value=3.0)
    target_y = st.sidebar.number_input("Target Y", value=2.0)

    if st.button("🚀 Start Training & Animation", type="primary"):
        
        st.markdown("### 🧠 Training Progress")
        progress_bar = st.progress(0)
        status_text = st.empty()

        with st.spinner('Training the algorithm... Please wait'):
            arm = RoboticArm(link_lengths)
            target_point = (target_x, target_y)
            
            ga = GeneticAlgorithm(arm=arm, target=target_point, population_size=pop_size, mutation_rate=mut_rate)
            
            best_angles, history = ga.run(
                generations=max_gen, 
                progress_bar=progress_bar, 
                status_text=status_text
            )

            max_reach = sum(link_lengths)
            plot_limit = max(max_reach + 1.0, max(abs(target_x), abs(target_y)) + 1.0)
            
            fig, ax = plt.subplots(figsize=(6, 6))
            ax.set_xlim(-plot_limit, plot_limit)
            ax.set_ylim(-plot_limit, plot_limit)
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.plot(target_point[0], target_point[1], 'r*', markersize=15)
            
            line, = ax.plot([], [], 'o-', lw=4, markersize=8, color='#38bdf8')
            generation_text = ax.text(-plot_limit + 0.5, plot_limit - 1.0, '', fontsize=12)

            def update(frame):
                joints = arm.forward_kinematics(history[frame])
                line.set_data([j[0] for j in joints], [j[1] for j in joints])
                generation_text.set_text(f'Generation: {frame}')
                return line, generation_text

            ani = animation.FuncAnimation(fig, update, frames=len(history), interval=100, blit=True, repeat=False)
            
            st.success(f"Training completed successfully! Total Generations: {max_gen}")
            html_anim = ani.to_jshtml()
            components.html(html_anim, height=700)
    

def check_streamlit():
    try:
        from streamlit.runtime.scriptrunner import get_script_run_ctx
        return get_script_run_ctx() is not None
    except ImportError:
        return False

if __name__ == "__main__":
    if check_streamlit():
        run_streamlit()
    else:
        run_local()
