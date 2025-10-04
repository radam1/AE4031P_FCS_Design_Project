%% Part 1-4: Setup and Declaration of Flight Conditions
% Based on the specifications in the assignment, we are given: 
alt = 10000; %ft
speed = 350; %ft

%Ensure there is a nlplant mex file for your system
mex nlplant.c

%% Part 5: Trim and Linearize the Model
% Using Low-Fidelity Model For the to Find a_n characteristics
% Getting the Transfer Function for elevator input to x_a
a_n_index = 19; 
x_as = [0, 5, 5.9, 6, 7, 15]; 
elev_index = 2; 

model = 'LIN_F16Block_With_an';
open_system(model);

% Path to the block you want to modify
blockPath = [model '/a_x'];

for idx=1:size(x_as, 2)

x_a=x_as(idx);
set_param(blockPath, 'Value', num2str(x_a));

close_system(model, 1);
open_system(model);

sprintf("Values for x_{a} = %3.2f ft", x_a);

%With given an, find the 
FindF16Dynamics_15000_500

%Grab State-Space Vectors
A = SS_lo.A; 
B = SS_lo.B; 
C = SS_lo.C; 
D = SS_lo.D; 

%Create state-space equations for a_n and elevator 
A_an = A;
B_an = B(:, elev_index); 
C_an = C(a_n_index, :); 
D_an = D(a_n_index, elev_index); 

if x_a == 0

end
%Convert ss->tf and show it
ss_an = ss(A_an,B_an,C_an, D_an); 
disp('Transfer function from elevator to a_n:');
an_tf = tf(ss_an)
disp("Transfer function's Zeros:");
zero(an_tf)

[tf_num, tf_denom] = ss2tf(A_an,B_an,C_an, D_an); 
an_tf = tf(tf_num, tf_denom);

%Find Dependencies and display ss-eqns
syms npos epos h phi theta psi Vt alpha beta p q r thrust_state  elevator_state aileron_state rudder_state Thrust_cmd Elevator_cmd Aileron_cmd Rudder_cmd
input_names = [npos; epos; h; phi; theta; psi; Vt; alpha; beta; p; q; r; thrust_state;  elevator_state; aileron_state; rudder_state]; 
output_names = [Thrust_cmd; Elevator_cmd; Aileron_cmd ;Rudder_cmd]; 

dependencies = input_names(not (C_an==0));
ss_an_elev = vpa(C_an * input_names + D(elev_index, :) * output_names, 3)

%Display Negative Step Response
figure()
times = linspace(0, 2, 200); 
u_step = ones(size(times)); 
lsim(-an_tf, u_step, times)
ylim([-.1, .8])
xlim([0,2])
title1 = sprintf("Step Response With x_{a} = %3.2f ft", x_a);
filename = "Figures/" + sprintf("step_x_a%2.1f", x_a) + ".pdf";
title(title1)
exportgraphics(gca,filename)

end


%% Part 6a: Open Loop Analysis of Low-Fidelity System 
%Switch to Given Flight Condition
FindF16Dynamics_10000_350

%% Part 6b (getting Eigenmotion Responses)
% Getting Lateral State-Space Matrices
%Grab State-Space Vectors
A = SS_lo.A; 
B = SS_lo.B; 
C = SS_lo.C; 
D = SS_lo.D; 

%Lateral States: 
% 
state_indeces_lat = [4 9 10 12 15 16]; 
state_names_lat = string(SS_lo.StateName(state_indeces_lat));
input_indeces_lat = [3 4]; 
input_names_lat = string(SS_lo.InputName(input_indeces_lat));

A_lat_reduced = A(state_indeces_lat, state_indeces_lat);
B_lat_reduced = B(state_indeces_lat, input_indeces_lat);
C_lat_reduced = C(state_indeces_lat, state_indeces_lat);
D_lat_reduced = zeros(size(B_lat_reduced)); 

A_ac_lat = A_lat_reduced(1:4, 1:4);
B_ac_lat = A_lat_reduced(1:4, 5:6);
C_ac_lat = C_lat_reduced(1:4, 1:4);
D_ac_lat = zeros(4,2);

lat_tfs = tf(ss(A_lat_reduced, B_lat_reduced, C_lat_reduced, D_lat_reduced)); %W/ actuators
lat_tfs_noAc = tf(ss(A_ac_lat, B_ac_lat, C_ac_lat, D_ac_lat)); % w/o actuators 

%Getting Longitudinal State-Space Matrices

state_indeces_long = [5 7 8 11 13 14];
state_names_long = string(SS_lo.StateName(state_indeces_long)); 
input_indeces_long = [1 2]; 
input_names_long = string(SS_lo.InputName(input_indeces_long));

A_long_reduced = A(state_indeces_long, state_indeces_long);
B_long_reduced = B(state_indeces_long, input_indeces_long);
C_long_reduced = C(state_indeces_long, state_indeces_long);
D_long_reduced = zeros(size(B_long_reduced)); 

A_ac_long = A_long_reduced(1:4, 1:4);
B_ac_long = A_long_reduced(1:4, 5:6);
C_ac_long = C_long_reduced(1:4, 1:4);
D_ac_long = zeros(4,2);
long_tfs = tf(ss(A_long_reduced, B_long_reduced, C_long_reduced, D_long_reduced)); 
long_tfs_noAc = tf(ss(A_ac_long, B_ac_long, C_ac_long, D_ac_long)); %Using
%Reduced Signal

%Extracting Longitudinal Eigenmodes
longitudinal_eig = eig(A_ac_long)
lateral_eig = eig(A_ac_lat)

%Derriving damping ratios, half-amplitude times for lateral and
%longitudinal eigenmodes
phugoid = min(longitudinal_eig); %Note that min/max measure magnitude in imaginary #s
phugoid_z = abs(phugoid)/(2 * abs(imag(phugoid)))
phugoid_omega = abs(phugoid)
phugoid_t12 = log(0.5)/(- phugoid_z * phugoid_omega)
%Plotting Phugoid: Elevator to V and to theta
v_elevator = long_tfs(2, 2); 
figure()
subplot(2, 1, 1)
step(v_elevator)
title("Elevator Defelction to Velocity")
xlabel("t")
ylabel("V[ft/s]")

theta_elevator = long_tfs(1, 2); 
subplot(2, 1, 2)
step(theta_elevator)
title("Elevator Defelction to Pitch")
xlabel("t")
ylabel("\theta [deg]")

%Short-Period Characteristics and Plots
short_period = max(longitudinal_eig);
sp_z = abs(short_period)/(2 * abs(imag(short_period)))
sp_omega = abs(short_period)
sp_t12 = log(0.5)/(- sp_z * sp_omega)
%Plotting Short-Period: Elevator to alpha and elevator to q 
alpha_elevator = long_tfs(3, 2); 
figure()
subplot(2, 1, 1)
step(alpha_elevator)
title("Elevator Defelction to Alpha")
xlabel("t")
ylabel("\alpha [deg]")

q_elevator = long_tfs(4, 2); 
subplot(2, 1, 2)
step(q_elevator)
title("Elevator Defelction to Pitch Rate")
xlabel("t")
ylabel("q [deg/s]")

%Dutch-Roll Characteristics and Plots
dutch_roll = lateral_eig(~imag(lateral_eig)==0); dutch_roll=dutch_roll(1);
dr_z = abs(dutch_roll)/(2 * abs(imag(dutch_roll)))
dr_omega = abs(dutch_roll)
dr_t12 = log(0.5)/(- dr_z * dr_omega)
%Plotting Dutch Roll: Lateral Rudder Impulse Responses for all states
figure()
for idx=1:4
    tf_i = lat_tfs(idx, 2); 
    subplot(2, 2, idx)
    impulseplot(tf_i)
    xlabel("t")
    ylabel(state_names_lat(idx))
    title(sprintf("%s Impulse Response to Aileron Deflection", state_names_lat(idx)))
end
sgtitle("Dutch Roll Responses")
%Aperiodic Roll Characteristics and Plots
aperiodic_roll = min(lateral_eig(imag(lateral_eig)==0));
aperiodic_thalf = log(0.5)/aperiodic_roll
%Plotting Aperiodic Roll: Aileron to Roll rate output
aileron_to_p = lat_tfs(3,1); 
figure()
step(aileron_to_p)
title("Aileron to Roll Rate(p) Step Response")
xlabel("time")
ylabel("p [deg/s]")

%Spiral Roll Characteristics and Plots
spiral_roll = max(lateral_eig(imag(lateral_eig)==0));
spiral_thalf = log(0.5)/spiral_roll
s = tf('s'); spiral_tf = 1/(s-spiral_roll); 

%Plotting Spiral Roll: Time Response to Initial Roll Angle
x0 = 20 * pi / 180; %20deg roll 
t = linspace(0, 800, 1000); % 350 seconds

%use initial() function to simulate system with initial condition 
[y, t_out, x] = initial(ss(spiral_tf), x0, t);

%Plot time response
figure()
plot(t_out, y(:, 1) * 180 / pi); % Convert radians to degrees
xlabel('Time (seconds)');
ylabel('\phi [deg]');
title('Roll Reponse With Initial 20° Roll Angle');

%% Part 7: Designing Pitch Rate Feedback Controller 
%Step 1: Constructing Reduced Transfer Function For Pitch-Rate Feedback
%States: Angle of Attack and pitch rate
state_indeces_long = [8 11 14]; 
state_names_long = string(SS_lo.StateName(state_indeces_long));
input_indeces_long = 2; 
input_names_long = string(SS_lo.InputName(input_indeces_long));

%Get the full 2-state transfer function with actuator dynamics
A_long_p4 = A(state_indeces_long, state_indeces_long);
B_long_p4 = B(state_indeces_long, input_indeces_long);
C_long_p4 = C(state_indeces_long, state_indeces_long);
D_long_p4 = zeros(size(state_indeces_long, 1), size(input_indeces_long,1));

%Reduce to only Angle of Attack and Pitch Rate 
A_ac_p4 = A_long_p4([1,2],[1,2]); 
B_ac_p4 = A_long_p4([1,2],3); 
C_ac_p4 = C_long_p4([1,2],[1,2]); 
D_ac_p4 = C_long_p4([1,2],3); 
lat_tfs_p4 = tf(ss(A_ac_p4, B_ac_p4,C_ac_p4,D_ac_p4))

%constructing the transfer function from Short-Period Pole 
s = tf("s"); 
sp_response_poles = sp_omega^2/(s^2 + 2 * sp_z * sp_omega * s + sp_omega^2)

q_elev_p4 = lat_tfs_p4(2); 
alpha_elev_p4 = lat_tfs_p4(1); 

%Load Numerator & Denominator for simulink
[num_sp, den_sp] = tfdata(q_elev_p4, 'v');
%Step 2: Comparing pitch rate of 4-state+actuator with 2-state without actuator
figure()
step(q_elevator)
hold on 
step(-q_elev_p4)
step(sp_response_poles)
hold off 
legend("4-state with actuator", "2-State Short-Period Response", "Derrived From Short-Period Poles")
title("Comparing Multiple Short-Period Damping Models")
%From Plot and control system logic, phugoid is dominant pole, as its
%response takes much longer to settle than the short-period.

%ACTUAL ANSWER TO Q7: 
%{ 
From the simulink diagram, you can see that zero-placement is how you move
the poles to exactly where they need to be for damping. The current
closed-loop zero is too close to the imaginary axis, so the poles bend
toward the imaginary axis instead of bending away from the imaginary axis.
By cancelling the real zero close to the imaginary axis and replacing it
with a real negative zero on the order of 10^12, the poles can be tuned to
the exact specifications
%} 

%% 
%Alternative: Create and tune a PI Controller
%THIS IS NOT USED. WE USE THE SIMULINK MODEL INSTEAD
alt = 10000; %ft
speed = 350; %ft

V = speed * 0.3048; %m/s
cn1 = -num_sp(2); cn2 = -num_sp(3); cd1 = den_sp(2); cd2 = den_sp(3); 
w_nsp_req = 0.03 * V
time_const = 1/(0.75 * w_nsp_req)
z_sp_req = 0.5
desiredPole = -z_sp_req * w_nsp_req + 1i *  w_nsp_req * sqrt(1-z_sp_req^2); a = real(desiredPole); b = imag(desiredPole); 
Amat = [1, cn1, 0; (a^2+b^2), 0, cn2; 2*a, cn2, cn1]
B = [cd1+2*a; 0;cd2-(a^2 + b^2)]
X = inv(Amat) * B; Kp = X(2); Ka = X(3); 

%Fine-tuning gains manually to achieve control parameters
Kp = Kp + (-0.25); 
Ka = Ka + (-0.65); 

figure()
rlocus(-q_elev_p4)
sgrid(z_sp_req, w_nsp_req)
title("Root Locus of Short-Period Response")


%Q3: Create PI controller for pitch rate: (note kp = -2.2 ki=-6.7 were
%manually tuned values) 
%Kp2 = (1.3 - 2 * a - 10)/3.76; %Manually Calculated for pole placement
%Ka2 = (-(a^2 + b^2) * 10 )/1.9; %Manually Calculated for pole placement
pitch_controller= Kp + Ka / tf('s'); 
[num_pc, den_pc] = tfdata(pitch_controller, 'v');
controlled_cl_tf = feedback(pitch_controller * q_elev_p4, 1, -1) % controlled closed-loop transfer function

%Plot the step response of feedback vs vs open-loop
figure()
step(controlled_cl_tf)
hold on 
step(-q_elev_p4)
hold off 
legend("PI Controlled Feedback System", "Open Loop")
title("Step Response of PI Controlled Short-Period System vs Open Loop")

%Q4: Check Designed PI Controller for Damping and Natural Frequency
cl_poles = pole(controlled_cl_tf);
sprintf("Information about closed-loop transfer function damping, natural Frequency, and tiem constant")
damp(cl_poles)

%{
Q5: The pole/zero-cancellation or lead-lag prefilter must be placed outside of
the feedback loop because it is trying to effect the closed-loop poles and
zeros. Placing it inside of the feedback loop would not cancel the pole
correctly, as the filter would be affected by the feedback dynamics. In
addition, the purpose of the lead-lag filter would be to adjust the signal
pre-feedback. 
%}

%% Q7: CAP, assuming damping and natural frequency of the complex poles
g = 9.81; %m/s^2
tau = time_const; %s control tuning app
wsp = w_nsp_req; %rad/s from requirements
z = 0.5; 
CAP = wsp^2 * g * tau /V 

%For Gibson Criterion
DB_qss = tau - 2 * z/wsp
overshoot = 1.15 % for using in the simulink control tuning app overshoot response

%Finally: Save workspace for Simunlink Model 
save("PitchController")

%% Getting some of the plots for the pitch rate controller
file = "q_alpha_controller_FullyFitting";
open_system(file)

%Now gather the outputs from the SimOutput struct
outputs = sim(file); 
tout = outputs.tout; 
controlled = outputs.controlled.Data; 
controlledNW = outputs.controlledNoWind.Data; 
uncontrolled = outputs.uncontrolled.Data; 
close_system(file, 1)

figure()
plot(tout, controlled)
hold on 
plot(tout, uncontrolled)
hold off 
legend("Controlled Feedback System", "Open Loop")
title("Step Response of Controlled Short-Period System vs Open Loop")

figure()
plot(tout, controlledNW)
hold on 
plot(tout, controlled) 
hold on 
legend("No Gust", "15ft/s Gust")
title("Step Response of Aircraft With and Without a Wind Gust")
exportgraphics(gca,'Figures/Gust_NoGust.pdf')

%% Part 8: Designing Glidescope/Flare Controller

%Step 1: Trim and Linearize about alt=

FindF16Dynamics_5000_300
alt = 5000; 
v = 300; 

A = SS_lo.A; 
B = SS_lo.B; 
C = SS_lo.C; 
D = SS_lo.D; 
%% Step 2: Reduce to necessary states: 
% alt h, airspeed v, angle of attack a, pitch attitude angle theta, and
% pitch rate q
state_indeces_long = [3 5 7 8 11 13 14]; 
state_names_long = string(SS_lo.StateName(state_indeces_long));
input_indeces_long = [1, 2]; 
input_names_long = string(SS_lo.InputName(input_indeces_long));

%Get the full 2-state transfer function with actuator dynamics
A_long_p8 = A(state_indeces_long, state_indeces_long);
B_long_p8 = B(state_indeces_long, input_indeces_long);
C_long_p8 = C(state_indeces_long, state_indeces_long);
D_long_p8 = zeros(size(state_indeces_long, 1), size(input_indeces_long,1));

%Reduce to only state, not actuator 
A_ac_p8 = A_long_p8(1:5,1:5); 
B_ac_p8 = A_long_p8(1:5,6:7); 
C_ac_p8 = C_long_p8(1:5,1:5); 
D_ac_p8 = C_long_p8(1:5,6:7); 

glidescope_statespace = ss(A_ac_p8, B_ac_p8,C_ac_p8,D_ac_p8);
glidescope_tf = tf(glidescope_statespace);
% Step 3-4 in Simulink: 
%Step 5: Additional Variables for the Controller
gamma_to_theta = 1- glidescope_tf(4,2)/glidescope_tf(2,2);
[num_gt, denom_gt] = tfdata(gamma_to_theta, "v"); 

Kv = 100; %Velocity Gain. Manually Tuned

%Flare Geometry Constants
x1 = 1000; %ft (Chosen Somewhat Arbitrarily)
v_touch = 2.5; %ft/s

syms hf h0 x2 tau T 
eq1 = hf - h0 == x2 * tand(3); 
eq2 = -v * sind(3) == -hf / tau; 
eq3 = hf * exp(-T/tau) - h0 == 0; 
eq4 = -(hf/tau) * exp(-T/tau) == -v_touch; 
eq5 = x1 + x2 == v * T; 

sol = vpasolve([eq1, eq2, eq3, eq4, eq5], [hf, h0, x2, tau, T]);

hf = double(sol.hf);
h0= double(sol.h0);
x2= double(sol.x2);
tau= double(sol.tau);
T = double(sol.T);

%Finally, Save Workspace so you can load it in later
save("glidescope")


%% Making plots from glidescope info 
%first load the workspace and run the simulation
load("glidescope")
file = "Glidescope_Flare_ActuallyGood";
open_system(file)

%Now gather the outputs from the SimOutput struct
outputs = sim(file); 
tout = outputs.tout; 
close_system(file, 1)

%State Variables/Derivatives
alpha = outputs.alpha.Data; 
pos = outputs.pos.Data; pos = max(pos) - pos; 
h = outputs.h.Data; 
q = outputs.q.Data; 
theta = outputs.theta.Data;
vt = outputs.vt.Data; 
vy = outputs.vy.Data; 

%Glidesscope-Specific Variables 
d = outputs.d.Data;
lambda = outputs.lambda.Data; 
 
%Control efforts
elevator = outputs.elevator.Data; 
thrust = outputs.thrust.Data; 

%Some things we need to calculate for graphing: 
%1) time of switchover 
indSwitch = h-hf == min(abs(h-hf)); 
tSwitch = tout(indSwitch); 

%2) Runway x Val(assumes runway starts at glidescope transmitter
runwayDist = pos(indSwitch) + hf/tand(3);

%Pt 1: Glidescope 
%1: Altitude vs Time
figure()
plot(tout, h)
title("Altitude of Glidescope Maneuver")
xlabel("Time [s]")
ylabel("Altitude [ft]")
exportgraphics(gca,'Figures/GS_Alt_v_Time.pdf')

%2: Speed vs Time
figure()
plot(tout, vt)
title("Velocity During Glidescope Descent")
xlabel("Time [s]")
ylabel("Velocity [ft/s]")
exportgraphics(gca,'Figures/GS_Speed_v_Time.pdf')

%3: Pitch vs Time
figure()
plot(tout, theta)
title("Pitch During Glidescope Descent")
xlabel("Time [s]")
ylabel("Pitch \theta [deg]")
exportgraphics(gca,'Figures/GS_Pitch_v_Time.pdf')

%4: AOA vs Time
figure()
plot(tout, alpha)
title("Angle of Attack During Glidescope Descent")
xlabel("Time [s]")
ylabel("Angle of Attack \alpha [deg]")
exportgraphics(gca,'Figures/GS_AOA_v_Time.pdf')

%5: Distance from GS vs Time
figure()
plot(tout, d)
title("Distance from GS Signal During Glidescope Descent")
xlabel("Time [s]")
ylabel("Distance From Glidescope [ft]")
exportgraphics(gca,'Figures/GS_Dist_v_Time.pdf')

%6: Range vs Altitude
figure()
plot(pos, h)
title("Altitude vs Range During Glidescope Descent")
xlabel("Range [ft]")
ylabel("Altitude [ft]")
exportgraphics(gca,'Figures/GS_Range_v_Alt.pdf')

%Pt 2: Flare 
figure()
plot(pos, h)
hold on 
plot([runwayDist, runwayDist], [0, hf+100], "--r")
hold off 
title("Altitude vs Range During Flare Maneuver")
xlabel("Range [ft]")
ylabel("Altitude [ft]")
ylim([0, hf+100])
xlim([pos(indSwitch), pos(end)])
legend("Aircraft Path", "Runway Start")
exportgraphics(gca,'Figures/Flare_Range_v_Alt.pdf')

%Vertical velocity during flare maneuver
figure()
plot(tout, vy)
hold on
plot([tout(indSwitch), tout(end)], [-2.5, -2.5], "--r")
title("Vertical Velocity During Flare Maneuver")
xlabel("Time [s]")
ylabel("Velocity [ft/s]")
xlim([tout(indSwitch), tout(end)])
legend("Vertical Velcoity", "Vertical Velocity Bound")
exportgraphics(gca,'Figures/Flare_Vert_Velocity.pdf')

%Mixed Plot of Pitch, AOA, and Velocity
figure()
subplot(3, 1, 1)
plot(tout, theta)
title("Pitch During Flare Maneuver")
xlabel("Time [s]")
ylabel("Pitch \theta [deg]")
xlim([tout(indSwitch), tout(end)])
ylim([-4, -0.5])

subplot(3, 1, 2)
plot(tout, alpha)
title("Angle of Attack During Flare Maneuver")
xlabel("Time [s]")
ylabel("Angle of Attack \alpha [deg]")
xlim([tout(indSwitch), tout(end)])
ylim([-1.5, 0.5])

subplot(3, 1, 3)
plot(tout, vt)
title("Velocity During Flare Maneuver")
xlabel("Time [s]")
ylabel("Velocity [ft/s]")
xlim([tout(indSwitch), tout(end)])
ylim([300, 315])
exportgraphics(gca,'Figures/Flare_Mixed_Plots.pdf')

