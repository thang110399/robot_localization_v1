from easymodbus.modbusClient import ModbusClient


class RS485:
    def __init__(self, port="/dev/ttyUSB-arduino2", is_rtu=True, ip=None):
        self.port = port
        self.ip = ip
        self.is_rtu = is_rtu
        self.connected_to_plc = False

    def check_connection(self):
        '''check connection to plc
        '''
        try:
            if self.is_rtu:
                plc = ModbusClient(f'/dev/ttyUSB-arduino{self.port}')
            else:
                plc = ModbusClient(self.ip, self.port)

            if not plc.is_connected():
                plc.connect()

            try:
                # test truyen nhan du lieu
                plc.write_single_coil(13, 1)
                verify_coil = (1 == int(plc.read_coils(13, 1)[0]))
                plc.write_single_register(13, 1)
                verify_reg = (1 == int(plc.read_holdingregisters(13, 1)[0]))
                self.connected_to_plc = (verify_coil and verify_reg)
            except Exception as e:
                print(e)
                self.connected_to_plc = False

            plc.close()

        except Exception as e:
            print(e)
            self.connected_to_plc = False

    def write(self, type_, address, value):
        '''write data to plc with type and address defined

        Args:
            type_ (str): type of place, available: reg, coil.
            address (int): address of where to write. eg. 1, 2, 3.
            value (int): value of data in decimal. eg. 1, 2, 3.
        '''
        try:
            #  check connection
            plc = None
            try:
                if self.is_rtu:
                    plc = ModbusClient(f'/dev/ttyUSB-arduino{self.port}')
                else:
                    plc = ModbusClient(self.ip, self.port)
            except Exception as e:
                print("Error", e)

            if not plc.is_connected():
                plc.connect()

            if type_ == 'coil':
                plc.write_single_coil(address, value)
            elif type_ == 'reg':
                plc.write_single_register(address, value)

            plc.close()

        except Exception as e:
            print(e)

    def read(self, type_, address):
        """read data from plc with type and address defined

        Args:
            type_ (str): type of reading functions. Available: hr, ir, coil.
            address (int): address of reading type. eg. 1, 2, 3.

        Returns:
            [list]: list of results
        """
        try:
            if self.is_rtu:
                plc = ModbusClient(f'/dev/ttyUSB-arduino{self.port}')
            else:
                plc = ModbusClient(self.ip, self.port)

            if not plc.is_connected():
                plc.connect()

            if type_.strip() == 'hr':
                results = plc.read_holdingregisters(address, 1)
            elif type_.strip() == 'ir':
                results = plc.read_inputregisters(address, 1)
            elif type_.strip() == 'coil':
                results = plc.read_coils(address, 1)
            else:
                raise Exception("Wrong type")

            plc.close()

            return int(results[0])

        except Exception as e:
            print(e)


if __name__ == '__main__':
    rs = RS485(port=2)
    
    rs.check_connection()
    print(rs.connected_to_plc)

    rs.write('coil', 101, 1)
    print(rs.read('coil', 101))

    # rs.write('reg', 220, 101)
    # for i in range(10):
    #     print(i, rs.read('hr', 220 + i))
    # print(rs.read('ir', 220))
    rs.write('reg', 50, 30)
    print(rs.read('hr', 50))