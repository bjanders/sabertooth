package sabertooth

import (
	"errors"

	"go.bug.st/serial"
	"go.bug.st/serial/enumerator"
)

// Sabertooth commands
const (
	CmdSet          = 40
	CmdSetValue     = 0
	CmdSetKeepalive = 16
	CmdSetShutdown  = 32
	CmdSetTimeout   = 64

	CmdGet        = 41
	CmdGetValue   = 0
	CmdGetBattery = 16
	CmdGetCurrent = 32
	CmdGetTemp    = 64

	CmdReply = 73
)

// Sabertooth represents a Sabertooth controllers
type Sabertooth struct {
	address  byte
	portName string
	port     serial.Port
}

// Packet is a the data sent or received from a Sabertooth
type Packet struct {
	Address byte
	Target  byte
	Type    byte
	Number  byte
	Value   int16
}

// NewSabertooth creates a new Sabertooth device. The default address is 128.
// The portName is the serial port where the device is attached. You
// se the SerialPort() function to find the USB serial port that the device
// is connected to.
func NewSabertooth(address byte, portName string) (*Sabertooth, error) {
	st := Sabertooth{}
	st.address = address
	st.portName = portName

	return &st, nil
}

// OpenPort opens the servial port
func (st *Sabertooth) OpenPort() error {
	mode := &serial.Mode{
		BaudRate: 115200,
	}
	var err error
	st.port, err = serial.Open(st.portName, mode)
	if err != nil {
		return err
	}
	return nil
}

// Input gets the input value of on any of the input ports of
// the device. port can be 'S', 'A', 'M' or 'P'. n can be 1 or 2.
// The returned value is between -1 and 1 inclusive.
func (st *Sabertooth) Input(port byte, n int) (float64, error) {
	value, err := st.Read(CmdGetValue, port, byte(n))
	if err != nil {
		return 0, err
	}
	return float64(value) / 2047, nil
}

// Battery returns the battery voltage
func (st *Sabertooth) Battery() (float64, error) {
	bat, err := st.Read(CmdGetBattery, 'M', 1)
	if err != nil {
		return 0, err
	}
	return float64(bat) / 10, nil
}

// Current returns the electirical current in Ampere of a motor driver
func (st *Sabertooth) Current(motor int) (float64, error) {
	current, err := st.Read(CmdGetCurrent, 'M', byte(motor))
	if err != nil {
		return 0, err
	}
	return float64(current) / 10, nil
}

// Temp returns the tempeture of a motor driver
func (st *Sabertooth) Temp(motor int) (int, error) {
	return st.Read(CmdGetTemp, 'M', byte(motor))
}

// Read reads of the parameters
func (st *Sabertooth) Read(param, target, number byte) (int, error) {
	if st.port == nil {
		err := st.OpenPort()
		if err != nil {
			return 0, err
		}
	}
	data := make([]byte, 9)
	n, err := st.port.Write(getCommand(st.address, param, target, number))
	if err != nil {
		return 0, err
	}
	n, err = st.port.Read(data)
	if err != nil {
		return 0, err
	}
	if n != 9 {
		return 0, errors.New("unexpected data length")
	}
	packet, err := decodePacket(data)
	if err != nil {
		return 0, err
	}
	return int(packet.Value), nil
}

// Motor controls the motors. motor is 1 or 2. speed is between -1 and 1
// inclusive
func (st *Sabertooth) Motor(motor int, speed float64) error {
	if speed < -1 || speed > 1 {
		return errors.New("value out of range")
	}
	value := speed * 2047
	n, err := st.port.Write(setCommand(st.address, CmdSetValue, 'M', byte(motor), int16(value)))
	if err != nil {
		return err
	}
	if n != 9 {
		return errors.New("wrote unexpected number of bytes")
	}
	return nil
}

// SerialPort scans the USB serial ports for a Sabertooth.
func SerialPort() (string, error) {
	ports, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return "", err
	}
	if len(ports) == 0 {
		return "", errors.New("no serial ports found")
	}

	portFound := false

	var portDetails *enumerator.PortDetails

	for _, portDetails = range ports {
		// fmt.Printf("Found port: %s\n", port.Name)
		if portDetails.IsUSB && portDetails.VID == "268B" && portDetails.PID == "0201" {
			//fmt.Printf("Found Sabertooth on port %s\n", portDetails.Name)
			portFound = true
			break
		}
	}
	if !portFound {
		return "", errors.New("sabertooth not found")
	}

	return portDetails.Name, nil
}

func makePacket(address, command, value byte, data []byte) []byte {
	size := 4
	if len(data) > 0 {
		size += len(data) + 1
	}
	packet := make([]byte, size)

	packet[0] = address
	packet[1] = command
	packet[2] = value
	packet[3] = (address + command + value) & 0x7f
	//packet[3] = crc7(packet[:3])

	if len(data) > 0 {
		var checksum byte
		for i := 0; i < len(data); i++ {
			packet[4+i] = data[i]
			checksum += data[i]
		}
		packet[4+len(data)] = checksum & 0x7f
	}

	return packet
}

func decodePacket(data []byte) (*Packet, error) {
	//log.Printf("%v", data)
	packet := Packet{}
	if data[1] != CmdReply {
		return nil, errors.New("unexpected command type")
	}
	packet.Address = data[0]
	packet.Value = int16(data[4]) + int16(data[5])<<7
	packet.Target = data[2]
	if packet.Target&1 == 1 {
		packet.Value = -packet.Value
		packet.Target--
	}
	packet.Type = data[6]
	packet.Number = data[7]

	return &packet, nil
}

func setCommand(address, setType, targetType, targetNumber byte, value int16) []byte {
	data := make([]byte, 4)

	data[2] = targetType
	data[3] = targetNumber
	if value < 0 {
		value = -value
		setType++
	}
	data[0] = byte(value & 0x7f)
	data[1] = byte((value >> 7) & 0x7f)
	return makePacket(address, CmdSet, setType, data)
}

func getCommand(address, getType, sourceType, sourceNumber byte) []byte {

	data := make([]byte, 2)
	data[0] = sourceType
	data[1] = sourceNumber
	return makePacket(address, CmdGet, getType, data)
}

func crc7(data []byte) byte {
	var crc byte = 0x7f
	for i := 0; i < len(data); i++ {
		crc ^= data[i]
		for bit := 0; bit < 8; bit++ {
			if crc&1 == 1 {
				crc >>= 1
				crc ^= 0x76
			} else {
				crc >>= 1
			}
		}
	}
	return crc ^ 0x7f
}
