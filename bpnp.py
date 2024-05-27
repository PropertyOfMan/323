import struct

import crc8

PACK_START = b'\x10'    # Байт начала пакета
ESCAPE_BYTE = b'\x10'   # Специальный байт
PACK_END = b'\x03'      # Байт конца пакета

BRT_ID = b'\xb0'    # ID Борта
FPB_ID = b'\xb1'    # ID Контроллера переднего силового блока
BPB_ID = b'\xb2'    # ID Контроллера заднего силового блока
CMC_ID = b'\xb3'    # ID Контроллера общего назначения

OPR_ID = b'\xa6'
RBT_ID = b'\xb0'

CD_ID = b'\x50'
NKR_ID = b'\x40'
AUD_ID = b'\x70'
BLE_ID = b'\x71'
LED_ID = b'\x41'
UAV_ID = b'\x42'
PTH_ID = b'\x51'
LID_ID = b'\x52'

ATM_ID = b'\xf1'
LTM_ID = b'\xf2'
FCE_ID = b'\x60'
STS_ID = b'\x61'
FND_ID = b'\x72'
TXT_ID = b'\x62'
CRD_ID = b'\x52'

WHU_ID = b'\x01'    # Запрос ID устройства
IAM_ID = b'\x02'    # ID устройства
ODO_ID = b'\xb1'    # Датчик скорости колеса
MR_ID = b'\xb2'     # Резольвер двигателя
MS_ID = b'\xb3'     # Скорость вала двигателя
MM_ID = b'\xba'     # Момент на двигателе (команда)
SM_ID = b'\xc0'     # Стоп сигнал вкл/выкл
RT_ID = b'\xc1'     # Правый поворотник вкл/выкл
LT_ID = b'\xc2'     # Левый поворотник вкл/выкл
ENC_ID = b'\xe0'    # Угол поворота рулевого шагового мотора
ST_ID = b'\xe1'     # Статус
RC_ID = b'\xea'     # Угол поворота шагового мотора (команда)
IMU_ID = b'\xb0'    # Показания инерциального модуля
UZ_ID = b'\xd0'     # Показания ультразвукового дальномера
SP_ID = b'\xbb'     # Гидравлический тормоз (ручник)

# ID сенсоров
FRONT_LEFT_WHEEL_SENSOR = 7
FRONT_RIGHT_WHEEL_SENSOR = 8
REAR_LEFT_WHEEL_SENSOR = 10
REAR_RIGHT_WHEEL_SENSOR = 11
FRONT_RESOLVER = 209
REAR_RESOLVER = 210
FRONT_MOTOR_SHAFT = 211
REAR_MOTOR_SHAFT = 212
FRONT_AXIS_ENC = 193
REAR_AXIS_ENC = 194
FRONT_AXIS_SWITCH = 195
REAR_AXIS_SWITCH = 196


SENDERS_BY_ID = {
    OPR_ID: 'OPERATOR',
    RBT_ID: 'ROBOT',

    BRT_ID: 'BORT',
    FPB_ID: 'FPU',
    BPB_ID: 'BPU',
    CMC_ID: 'CMC'
}

PACKS_BY_ID = {
    CD_ID: {'name': 'CD', 'format': '<B2f', 'type': 'CRD'},  # count * uchar 2*float            COUNT
    NKR_ID: {'name': 'NKR', 'format': '<2f', 'type': 'CSP'},  # 2*float
    AUD_ID: {'name': 'AUD', 'format': '<B', 'type': 'CSP'},  # uchar
    BLE_ID: {'name': 'BLE', 'format': '<6c', 'type': 'MAC'},  # count * 6*char                  COUNT
    LED_ID: {'name': 'LED', 'format': '<B', 'type': 'CSP'},  # uchar
    UAV_ID: {'name': 'UAV', 'format': '<4b', 'type': 'CSP'},  # 4*uchar
    PTH_ID: {'name': 'PTH', 'format': '<B2f', 'type': 'CRD'},  # count * uchar 2*float          COUNT
    LID_ID: {'name': 'LID', 'format': '<c', 'type': 'TXT'},  # count * char                     COUNT       INCORRECT

    ATM_ID: {'name': 'ATM', 'format': '<fBbI4e', 'type': 'CSP'},  # float uchar char uint 4*2bfloat
    LTM_ID: {'name': 'LTM', 'format': '<2f2bebB', 'type': 'CSP'},  # 2*float 2*char 2bfloat char uchar
    FCE_ID: {'name': 'FCE', 'format': '<4eB', 'type': 'CSP'},  # 4*2bfloat uchar
    STS_ID: {'name': 'STS', 'format': '<11b', 'type': 'STS'},  # 11*char
    FND_ID: {'name': 'FND', 'format': '<6c', 'type': 'MAC'},  # count * 6*char                  COUNT
    TXT_ID: {'name': 'TXT', 'format': '<c', 'type': 'TXT'},  # count * char                     COUNT
    CRD_ID: {'name': 'CRD', 'format': '<B2f', 'type': 'CRD'},  # count * uchar 2*float          COUNT


    WHU_ID: {'name': 'WHU', 'length': 0, 'format': '<', 'type': 'BLC'},
    IAM_ID: {'name': 'IAM', 'length': 0, 'format': '<', 'type': 'BLC'},
    ODO_ID: {'name': 'ODO', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    MR_ID: {'name': 'MR', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    MS_ID: {'name': 'MS', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    MM_ID: {'name': 'MM', 'length': 5, 'format': '<Bf', 'type': 'MM'},
    SM_ID: {'name': 'SM', 'length': 1, 'format': '<B', 'type': 'CMD'},
    RT_ID: {'name': 'RT', 'length': 1, 'format': '<B', 'type': 'CMD'},
    LT_ID: {'name': 'LT', 'length': 1, 'format': '<B', 'type': 'CMD'},
    ENC_ID: {'name': 'ENC', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    ST_ID: {'name': 'ST', 'length': 9, 'format': '<BLI', 'type': 'SID'},
    RC_ID: {'name': 'RC', 'length': 5, 'format': '<Bf', 'type': 'MM'},
    IMU_ID: {'name': 'IMU', 'length': 22, 'format': '<9hI', 'type': 'IMU'},
    UZ_ID: {'name': 'UZ', 'length': 9, 'format': '<BfI', 'type': 'SID'},
    SP_ID: {'name': 'SP', 'length': 1, 'format': '<B', 'type': 'CMD'}
}


# additional packs may be added to the dicts on the top. Further code should not be touched


PACKS_BY_NAMES = dict()

for ID in PACKS_BY_ID:
    PACKS_BY_NAMES[PACKS_BY_ID[ID]['name']] = ID


# Step 1: Find byte string and extract tuple: (<pack_name>, <body_byte_string>)
class PackageFinder():
    from enum import Enum

    class Status(Enum):
        WAITING_FOR_START = 0
        COLLECTING = 1
        AFTER_ESCAPE_CHAR = 2

    def __init__(self):
        self.status = self.Status.WAITING_FOR_START
        self.string = b''

    def _reset_(self):
        self.status = self.Status.WAITING_FOR_START
        self.string = b''

    def _calculate_crc_(self, byte_string):
        hash = crc8.crc8()
        hash.update(byte_string)
        return hash.digest()

    def check_byte(self, byte):
        if self.status == self.Status.WAITING_FOR_START:
            # check if byte is equal to \x10
            if byte == PACK_START:
                self.string += byte
                self.status = self.Status.COLLECTING
            return None

        if self.status == self.Status.COLLECTING:
            # while bytes is equal to \x10 do nothing, else start collecting string
            if byte != ESCAPE_BYTE:
                self.string += byte
            else:
                self.status = self.Status.AFTER_ESCAPE_CHAR
            return None

        if self.status == self.Status.AFTER_ESCAPE_CHAR:
            # if second byte wasn't equal to \x10, start collecting string
            if byte == ESCAPE_BYTE:
                self.string += byte
                self.status = self.Status.COLLECTING
                return None
            elif byte == PACK_END:
                if len(self.string) < 4:
                    self._reset_()
                    return (None, 'TOO SHORT STRING')
                pack = self.string[1:2]
                pack = {'name': None} if pack not in PACKS_BY_ID else PACKS_BY_ID[pack]
                data = self.string[1:-1]
                crc = self._calculate_crc_(self.string[:-1])
                pack_crc = self.string[-1:]
                self._reset_()
                if crc != pack_crc:
                    return (None, 'WRONG CRC')
                return (pack['name'], data)
            else:
                self._reset_()
                self.string += ESCAPE_BYTE
                self.string += byte
                self.status = self.Status.COLLECTING




# Step 2: get value from body byte string
class DataExtracter():

    def __init__(self, data):
        self.pack, self.body = data
        if self.pack in [name for name in PACKS_BY_NAMES]:
            self.pack = PACKS_BY_ID[PACKS_BY_NAMES[self.pack]]


    def extract(self):
        if self.pack == None or self.pack == 'LID':
            return 'UNDEFINED PACK TYPE'
        else:
            if self.pack['name'] in ['CD', 'BLE', 'PTH', 'LID', 'FND', 'TXT', 'CRD']:
                if self.pack['type'] == 'TXT':
                    tmp = []
                    for i in range(struct.unpack('<B', self.body[2:3])[0]):
                        tmp.append(struct.unpack(self.pack['format'],
                                                 self.body[3 + i:4 + i]))
                    return tmp

                if self.pack['type'] == 'CRD':
                    counter = 9

                if self.pack['type'] == 'MAC':
                    counter = 6

                tmp = []
                for i in range(struct.unpack('<B', self.body[2:3])[0]):
                    tmp.append(struct.unpack(self.pack['format'], self.body[3 + counter * i:3 + counter + counter * i]))
                return tmp

            else:
                return struct.unpack(self.pack['format'], self.body[2:])


# Step 1: pack values to body
class DataPacker():

    def __init__(self, sender):
        # Sender

        if sender == 'OPERATOR':
            self.sender_id = OPR_ID
        if sender == 'ROBOT':
            self.sender_id = RBT_ID


    def _binary_length_(self, integer_length):
        return integer_length.to_bytes(1, byteorder='little')


    def pack_msg(self, type, data):
        self.data = data
        self.type = type
        if type in PACKS_BY_NAMES:
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'CRD':
                return self._pack_count_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'TXT':
                return self._pack_count_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'MAC':
                return self._pack_count_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'STS':
                return self._pack_uncount_()
            if PACKS_BY_ID[PACKS_BY_NAMES[type]]['type'] == 'CSP':
                return self._pack_uncount_()
        else:
            print(type)
            raise Exception(f'Wrong message type. "{type}" is no valid message type.')


    def _pack_uncount_(self):
        return PACKS_BY_NAMES[self.type] + self.sender_id + struct.pack(PACKS_BY_ID[PACKS_BY_NAMES[self.type]]['format'], *self.data)


    def _pack_count_(self):
        type = PACKS_BY_ID[PACKS_BY_NAMES[self.type]]['type']
        if type == 'CRD':
            body = b''
            counter = 3

            cnt = 0
            for i in range(0, len(self.data), counter):
                body += struct.pack(PACKS_BY_ID[PACKS_BY_NAMES[self.type]]['format'], *self.data[i:i + counter])
                cnt += 1

            return PACKS_BY_NAMES[self.type] + self.sender_id + self._binary_length_(cnt) + body

        if type == 'MAC':
            body = b''
            counter = 6


            cnt = 0
            for i in range(0, len(self.data), counter):
                for elem in self.data[i:i + counter]:
                    body += int(elem, 16).to_bytes(1, byteorder='little')
                cnt += 1

            return PACKS_BY_NAMES[self.type] + self.sender_id + self._binary_length_(cnt) + body

        if type == 'TXT':
            return PACKS_BY_NAMES[self.type] + self.sender_id + self._binary_length_(len(self.data[0])) + (self.data[0]).encode()



# Step 2: compile a full package byte line
class PackagePacker():

    def _calculate_crc_(self, byte_string):
        hash = crc8.crc8()
        hash.update(byte_string)
        return hash.digest()

    def pack(self, line):
        return PACK_START + (line + self._calculate_crc_(PACK_START + line)).replace(b'\x10', b'\x10\x10') + ESCAPE_BYTE + PACK_END



if __name__ == '__main__':
    # form = lambda x: ' '.join(format(byte, '02x') for byte in x)
    #
    # bytes = [b'\x10\x50\xa6\x03\x00\xf1\x12\x01\x00\x00\x23\x65\xf1\x01\x00\xb0\xa0\xc2\x01\x80\xd1\xf0\x02\xe0\x00\x10\x10\x45\x09\x35\xb3\xf5\xb9\x10\x03',
    #          b'\x10\xf1\xb0\x43\x16\x19\x9a\x0c\x05\x00\x00\x4c\x5d\x03\xb9\x01\xc2\x02\x12\xff\xd3\xe9\x10\x03',
    #          b'\x10\x70\xa6\x04\x7a\x10\x03',
    #          b'\x10\x60\xb0\x04\x4a\x00\x38\x02\x8a\x02\xbc\x7b\xcb\x10\x03',
    #          b'\x10\x41\xa6\x01\xeb\x10\x03',
    #          b'\x10\xf2\xb0\x42\x5f\x03\xfe\x42\x16\x7b\xb3\x08\x00\x02\xfd\x19\x3c\x11\x10\x03',
    #          b'\x10\x40\xa6\x41\x08\xf5\xc3\x41\x7a\x14\x7b\xff\x10\x03',
    #          b'\x10\x42\xa6\x4b\x00\xce\x0f\x48\x10\x03',
    #          b'\x10\x71\xa6\x03\x8a\x98\x11\x8b\xa7\xfe\x4a\x8c\x5a\x6b\x00\x30\x7b\x1f\x8f\x09\x51\x1c\x90\x10\x03',
    #          b'\x10\x61\xb0\x01\x01\x01\x01\x00\x01\x01\x00\x01\x01\x01\xd4\x10\x03',
    #          b'\x10\x62\xb0\x03\x5b\xfe\x73\x63\x10\x03']
    # # 51 52 72
    # results = list()
    # finder = PackageFinder()
    # for j in range(len(bytes)):
    #     for i in range(len(bytes[j])):
    #         result = finder.check_byte(bytes[j][i:i+1])
    #     results.append(result)
    #
    #     extracter = DataExtracter(result).extract()
    #     print(result, '\n', extracter, '\n\n')

    # CD_ID: {'name': 'CD', 'format': '<B2f', 'type': 'CRD'},  # count * uchar 2*float            COUNT
    # NKR_ID: {'name': 'NKR', 'format': '<2f', 'type': 'CSP'},  # 2*float
    # AUD_ID: {'name': 'AUD', 'format': '<B', 'type': 'CSP'},  # uchar
    # BLE_ID: {'name': 'BLE', 'format': '<6c', 'type': 'MAC'},  # count * 6*char                  COUNT
    # LED_ID: {'name': 'LED', 'format': '<B', 'type': 'CSP'},  # uchar
    # UAV_ID: {'name': 'UAV', 'format': '<4b', 'type': 'CSP'},  # 4*uchar
    # PTH_ID: {'name': 'PTH', 'format': '<B2f', 'type': 'CRD'},  # count * uchar 2*float          COUNT
    # LID_ID: {'name': 'LID', 'format': '<c', 'type': 'TXT'},  # count * char                     COUNT       INCORRECT
    #
    # ATM_ID: {'name': 'ATM', 'format': '<fBbI4e', 'type': 'CSP'},  # float uchar char uint 4*2bfloat
    # LTM_ID: {'name': 'LTM', 'format': '<2f2bebB', 'type': 'CSP'},  # 2*float 2*char 2bfloat char uchar
    # FCE_ID: {'name': 'FCE', 'format': '<4eB', 'type': 'CSP'},  # 4*2bfloat uchar
    # STS_ID: {'name': 'STS', 'format': '<11b', 'type': 'STS'},  # 11*char
    # FND_ID: {'name': 'FND', 'format': '<6c', 'type': 'MAC'},  # count * 6*char                  COUNT
    # TXT_ID: {'name': 'TXT', 'format': '<c', 'type': 'TXT'},  # count * char                     COUNT
    # CRD_ID: {'name': 'CRD', 'format': '<B2f', 'type': 'CRD'},  # count * uchar 2*float          COUNT

    inputs = [['OPERATOR', 'CD', [0, 1.4, 1.7, 1, 1.1, 1.1, 2, 500, 51.00001]],
              ['ROBOT', 'NKR', [123.321, 321.123]],
              ['OPERATOR', 'AUD', [111]],
              ['ROBOT', 'BLE', ['8e', '9f', '11', '13', '11', '00']],
              ['OPERATOR', 'LED', [1]],
              ['OPERATOR', 'UAV', [11, 12, 1, 120]],
              ['OPERATOR', 'PTH', [12, 123.11, 0.0001, 42, -123, -1, 4, 123, 123, 53, 11, 9999]],

              ['OPERATOR', 'ATM', [123.312, 55, -55, 1423, 124.5, 55.534, 53.534, -55.55]],
              ['OPERATOR', 'LTM', [1252.2, 5555.5, -12, 15, 12.2, -4, 4]],
              ['OPERATOR', 'FCE', [12.3, 11.1, 55.5, -11.1, 12]],
              ['OPERATOR', 'STS', [1, 2, 2, 1, 0, 0, 1, 2, 1, 0, 1]],
              ['OPERATOR', 'FND', ['8e', '9f', '11', '13', '11', '00', '11', '22', '33', '44', '55', '66', '00', 'ff', 'ee', 'dd', 'cc', 'bb']],
              ['OPERATOR', 'TXT', ['Hello world!']],
              ['OPERATOR', 'CRD', [12, 123.11, 0.0001, 42, -123, -1, 4, 123, 123, 53, 11, 9999]]]
    # print(*results, sep='\n')
    # a = 'a'
    # print(int(a.encode().hex(), 16).to_bytes(1, byteorder='little'))
    finder = PackageFinder()
    a = 3
    print()
    for inp in inputs:
        packer = DataPacker(inp[0]).pack_msg(*inp[1:])
        pck = PackagePacker().pack(packer)
        for i in range(len(pck)):
            result = finder.check_byte(pck[i:i + 1])
        extracter = DataExtracter(result).extract()
        for elem in extracter:
            print(elem)
        print('\n-----------------------------\n')