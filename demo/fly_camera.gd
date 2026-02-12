extends Camera3D

## Fly-mode camera: WASD + mouse look
## Shift = fast, Scroll = adjust speed, Right-click to capture mouse

@export var move_speed := 5.0
@export var fast_multiplier := 3.0
@export var sensitivity := 0.002

var _velocity := Vector3.ZERO
var _yaw := 0.0
var _pitch := 0.0
var _captured := false

func _ready():
	# Initialize from current transform
	_yaw = rotation.y
	_pitch = rotation.x

func _unhandled_input(event: InputEvent):
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			if event.pressed:
				Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
				_captured = true
			else:
				Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
				_captured = false
		# Scroll to adjust speed
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			move_speed *= 1.2
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			move_speed /= 1.2

	if event is InputEventMouseMotion and _captured:
		_yaw -= event.relative.x * sensitivity
		_pitch -= event.relative.y * sensitivity
		_pitch = clamp(_pitch, -PI * 0.49, PI * 0.49)
		rotation = Vector3(_pitch, _yaw, 0)

func _process(delta: float):
	if not _captured:
		return

	var input := Vector3.ZERO
	if Input.is_key_pressed(KEY_W): input.z -= 1
	if Input.is_key_pressed(KEY_S): input.z += 1
	if Input.is_key_pressed(KEY_A): input.x -= 1
	if Input.is_key_pressed(KEY_D): input.x += 1
	if Input.is_key_pressed(KEY_E) or Input.is_key_pressed(KEY_SPACE): input.y += 1
	if Input.is_key_pressed(KEY_Q) or Input.is_key_pressed(KEY_CTRL): input.y -= 1

	if input.length() > 0:
		input = input.normalized()

	var speed = move_speed
	if Input.is_key_pressed(KEY_SHIFT):
		speed *= fast_multiplier

	translate(input * speed * delta)
