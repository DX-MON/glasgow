__all__ = ('to_int8', 'to_in32_be', 'decode_device_id_code')

def to_int8(bytes):
	return bytes[0]

def to_int32_be(bytes):
	return (
		(bytes[0] << 24) |
		(bytes[1] << 16) |
		(bytes[2] << 8) |
		bytes[3]
	)

jedec_id = {
	0x1f: 'Atmel'
}

avr_idcode = {
	0x9642: 'ATXMega64A3U',
	0x9742: 'ATXMega128A3U',
	0x9744: 'ATXMega192A3U',
	0x9842: 'ATXMega256A3U',
}

def decode_device_id_code(bytes):
	idCode = to_int32_be(bytes)
	version = (idCode >> 28) & 0xF
	part = (idCode >> 12) & 0xFFFF
	part = avr_idcode.get(part, f'{part:#x}')
	manufacturer = jedec_id.get((idCode >> 1) & 0x7FF, 'INVALID')
	id_hex = f'{idCode:#010x}'
	return id_hex, manufacturer, version, part
