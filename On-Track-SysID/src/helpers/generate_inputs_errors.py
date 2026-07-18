import torch

def generate_inputs_errors(v_y_next_pred, omega_next_pred, data):
  """
  Generate inputs and errors for the neural network.

  Computes errors between predicted and actual lateral velocity (v_y) and yaw rate (omega).
  Constructs input tensors for the neural network from the input data.

  Args:
      v_y_next_pred (numpy.ndarray): Predicted lateral velocity for the next timestep.
      omega_next_pred (numpy.ndarray): Predicted yaw rate for the next timestep.
      data (numpy.ndarray): Input data array with shape (n_samples, 4).

  Returns:
      tuple: Tuple containing input tensor and error tensor.
  """
  # Make copies of the input arrays
  v_y_next_pred = v_y_next_pred.copy()
  omega_next_pred = omega_next_pred.copy()
  v_x = data[:, 0].copy()
  v_y = data[:, 1].copy()
  omega = data[:, 2].copy()
  delta = data[:, 3].copy()

  # Convert to PyTorch tensors if not already tensors
  v_y_next_pred = torch.tensor(v_y_next_pred, dtype=torch.float32)
  omega_next_pred = torch.tensor(omega_next_pred, dtype=torch.float32)
  v_x = torch.tensor(v_x, dtype=torch.float32)
  v_y = torch.tensor(v_y, dtype=torch.float32)
  omega = torch.tensor(omega, dtype=torch.float32)
  delta = torch.tensor(delta, dtype=torch.float32)

  # Compute errors between predicted and actual v_y and omega
  error_v_y = (v_y[1:] - v_y_next_pred)
  error_omega = (omega[1:] - omega_next_pred)

  # Assemble inputs tensor directly from the already tensor-typed variables
  inputs = torch.stack([v_x[:-1], v_y[:-1], omega[:-1], delta[:-1]], dim=1)
  errors = torch.stack([error_v_y, error_omega], dim=1)
  return inputs, errors


def _window_one_segment(inputs_seg, errors_seg, window_size):
  """Slide a window of `window_size` timesteps (stride 1) over one contiguous,
  physically-continuous segment. Returns (X_seq, y_seq): X_seq shape
  (T-window_size+1, window_size, 4), y_seq shape (T-window_size+1, 2) - the
  residual target aligned to each window's LAST timestep (many-to-one).
  """
  T = inputs_seg.shape[0]
  if T < window_size:
    raise ValueError(f"segment length {T} < window_size {window_size} - increase "
                      "data_collection_duration or lower s4.sequence_length")
  X_seq = inputs_seg.unfold(0, window_size, 1).permute(0, 2, 1).contiguous()
  y_seq = errors_seg[window_size - 1:]
  return X_seq, y_seq


def generate_sequence_windows(inputs, errors, window_size, segment_len):
  """Build sliding-window sequences for the S4D temporal-residual model.

  `inputs`/`errors` are the flat, memoryless arrays generate_inputs_errors()
  already returns - `process_data()` (train_model.py) concatenates two
  physically-continuous segments (the filtered trace + its sign-mirrored
  copy, each of length `segment_len`), with exactly one corrupted boundary
  sample between them (index `segment_len-1` predicts across the mirror
  discontinuity). Each segment is windowed INDEPENDENTLY and that boundary
  sample is dropped entirely, rather than letting a window straddle it.
  """
  seg_a_in, seg_a_err = inputs[:segment_len - 1], errors[:segment_len - 1]
  seg_b_in, seg_b_err = inputs[segment_len:2 * segment_len - 1], errors[segment_len:2 * segment_len - 1]
  X_a, y_a = _window_one_segment(seg_a_in, seg_a_err, window_size)
  X_b, y_b = _window_one_segment(seg_b_in, seg_b_err, window_size)
  return torch.cat([X_a, X_b], dim=0), torch.cat([y_a, y_b], dim=0)