extends CharacterBody3D

## FPS walking camera: right-click to capture, WASD to walk, Space to jump, scroll to adjust speed.

@export var move_speed := 5.0
@export var fast_multiplier := 3.0
@export var sensitivity := 0.002
@export var jump_velocity := 4.5

var _yaw := 0.0
var _pitch := 0.0
var _captured := false
var _camera: Camera3D
var _gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")

func _ready():
	_yaw = rotation.y
	_pitch = 0.0

	# Collision capsule: GoldSrc player is 72 units tall, 16 unit radius
	var shape = CapsuleShape3D.new()
	shape.height = 1.8
	shape.radius = 0.4
	var col = CollisionShape3D.new()
	col.shape = shape
	add_child(col)

	# Camera at eye height (slightly below top of capsule)
	_camera = Camera3D.new()
	_camera.position.y = 0.7  # 0.9 (half height) - 0.2 offset = eye level
	_camera.current = true
	add_child(_camera)

	# CharacterBody3D settings for stairs
	floor_snap_length = 0.5
	floor_max_angle = deg_to_rad(50)

func _unhandled_input(event: InputEvent):
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			if event.pressed:
				Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
				_captured = true
			else:
				Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
				_captured = false
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			move_speed *= 1.2
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			move_speed /= 1.2

	if event is InputEventMouseMotion and _captured:
		_yaw -= event.relative.x * sensitivity
		_pitch -= event.relative.y * sensitivity
		_pitch = clamp(_pitch, -PI * 0.49, PI * 0.49)
		rotation.y = _yaw
		_camera.rotation.x = _pitch

func _physics_process(delta: float):
	# Gravity
	if not is_on_floor():
		velocity.y -= _gravity * delta

	# Jump
	if _captured and Input.is_key_pressed(KEY_SPACE) and is_on_floor():
		velocity.y = jump_velocity

	if not _captured:
		velocity.x = 0
		velocity.z = 0
		move_and_slide()
		return

	# WASD input
	var input := Vector2.ZERO
	if Input.is_key_pressed(KEY_W): input.y -= 1
	if Input.is_key_pressed(KEY_S): input.y += 1
	if Input.is_key_pressed(KEY_A): input.x -= 1
	if Input.is_key_pressed(KEY_D): input.x += 1
	input = input.normalized()

	var speed = move_speed
	if Input.is_key_pressed(KEY_SHIFT):
		speed *= fast_multiplier

	# Move relative to facing direction (horizontal only)
	var forward = -transform.basis.z
	forward.y = 0
	forward = forward.normalized()
	var right = transform.basis.x
	right.y = 0
	right = right.normalized()

	velocity.x = (forward * input.y + right * input.x).x * speed
	velocity.z = (forward * input.y + right * input.x).z * speed

	move_and_slide()
