# Deep Dynamics – Neural Vehicle Dynamics Modeling & Control

This repository provides a full pipeline for learning vehicle dynamics from real racing data and deploying the learned model inside a closed-loop NMPC (Nonlinear Model Predictive Control) simulation.

The workflow includes:
- Data preprocessing from raw driving logs
- Training a GRU-based deep dynamics model
- Open-loop evaluation of prediction accuracy
- Closed-loop NMPC simulation using the learned dynamics

---

## 1. Environment Setup & Dependencies

### Clone the Main Repository
```bash
git clone [https://github.com/eabdelghany/deep-dynamics.git](https://github.com/ebrahimabdelghfar/Ebrahim_Master_Thesis_repo.git)
cd deep-dynamics
```
Create and Activate Conda Environment

```bash
conda create -n deep_dynamics python=3.10 -y
conda activate deep_dynamics
```
Then install:

```bash
pip install -r requirements.txt
```

2. Data Preparation
Create Data Directory if not there
```bash
mkdir -p data
```
Move Raw Driving Logs

Assuming your raw file is named drive_log.csv:
```bash
mv ~/Downloads/drive_log.csv data/
```
Preprocess Data for GRU Training

This step converts flat CSV logs into fixed-length temporal windows:

```bash
python3 scripts/preprocess_data.py \
  --input data/drive_log.csv \
  --output data/processed_data.npz \
  --window_size 5
```
3. Training the Deep Dynamics Model
Configure Model Hyperparameters

Edit the model configuration file:

```bash
nano cfgs/model/deep_dynamics.yaml
```
You can tune:

Learning rate

GRU hidden size

Number of layers

Training epochs

Train the Model

```bash
python3 scripts/train.py --config cfgs/model/deep_dynamics.yaml
```
Trained weights will be saved under:
```bash
output/deep_dynamics/<experiment_name>/
```
4. Open-Loop Evaluation

Evaluate state prediction accuracy against ground-truth sensor data.

```bash
python3 scripts/evaluate.py \
  --model_path output/deep_dynamics/your_experiment/epoch_XXX.pth
```
Replace XXX with the epoch number of your best checkpoint.

5. Closed-Loop NMPC Simulation
Navigate to MPC Directory

```bash
cd deep_dynamics/mpc
```

Set Python Path

```bash
export PYTHONPATH=$PYTHONPATH:$(pwd)/../..
```
Run NMPC Simulation

Ensure the script points to your trained .pth model: (IMPORTANT)

```bash
python3 run_nmpc_orca_deep_dynamics.py
```
This runs a real-time closed-loop NMPC simulation using the learned vehicle dynamics.


