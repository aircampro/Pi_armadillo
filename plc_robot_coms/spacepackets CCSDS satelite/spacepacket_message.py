#!/usr/bin/python
#
# This shows how to use the spacepacket libraries for satelite communication 
# they are found at the ref:- https://github.com/us-irs/spacepackets-py
# various other tests are shown under here https://github.com/us-irs/spacepackets-py/blob/main/test
# and the libraries for composing each packet of data are here :- https://github.com/us-irs/spacepackets-py/tree/main/src/spacepackets
#
from unittest import TestCase
import struct
import fastcrc

from spacepackets import PacketType, SequenceFlags, SpacePacketHeader
from spacepackets.ccsds import PacketId, PacketSeqCtrl
from spacepackets.countdown import Countdown

from spacepackets.ccsds.spacepacket import (
    SpacePacket,
    get_apid_from_raw_space_packet,
    get_sp_packet_id_raw,
    get_sp_psc_raw,
    get_space_packet_id_bytes,
)

from spacepackets.uslp.defs import (
    UslpFhpVhopFieldMissingError,
    UslpInvalidConstructionRulesError,
    UslpInvalidFrameHeaderError,
    UslpTruncatedFrameNotAllowedError,
)
from spacepackets.uslp.frame import (
    FixedFrameProperties,
    FrameType,
    TfdzConstructionRules,
    TransferFrame,
    TransferFrameDataField,
    UslpProtocolIdentifier,
    VarFrameProperties,
)
from spacepackets.uslp.header import (
    BypassSequenceControlFlag,
    HeaderType,
    PrimaryHeader,
    ProtocolCommandFlag,
    SourceOrDestField,
    TruncatedPrimaryHeader,
    UslpInvalidRawPacketOrFrameLenError,
    UslpTypeMissmatchError,
    UslpVersionMissmatchError,
    determine_header_type,
)

class TestSpacePacket(TestCase):
    def setUp(self) -> None:
        self.sp_header = SpacePacketHeader(
            apid=0x02,
            data_len=0x16,
            seq_count=0x34,
            sec_header_flag=True,
            packet_type=PacketType.TC,
            seq_flags=SequenceFlags.FIRST_SEGMENT,
        )

    def test_basic(self):
        self.assertEqual(self.sp_header.apid, 0x02)
        self.assertEqual(self.sp_header.seq_flags, SequenceFlags.FIRST_SEGMENT)
        self.assertEqual(self.sp_header.ccsds_version, 0b000)
        self.assertEqual(self.sp_header.packet_id, PacketId(PacketType.TC, True, 0x02))
        self.assertEqual(
            self.sp_header.packet_seq_control,
            PacketSeqCtrl(SequenceFlags.FIRST_SEGMENT, 0x34),
        )
        self.assertEqual(self.sp_header.seq_count, 0x34)
        self.assertEqual(self.sp_header.data_len, 0x16)
        self.assertEqual(self.sp_header.packet_type, PacketType.TC)
        test_dict = {self.sp_header: 1}
        test_dict.update({self.sp_header: 2})
        self.assertEqual(test_dict[self.sp_header], 2)

    def test_basic_countdown(self, tm=50):
        test_cd = Countdown.from_millis(tm)
        self.assertTrue(test_cd.busy())
        self.assertFalse(test_cd.timed_out())
        self.assertTrue(test_cd.remaining_time().total_seconds() * 1000 > 40)
        time.sleep(0.05)
        self.assertTrue(test_cd.timed_out())
        self.assertTrue(test_cd.remaining_time() == timedelta())
        test_cd.timeout = timedelta(seconds=0.1)
        self.assertEqual(test_cd.timeout.total_seconds(), timedelta(seconds=0.1).total_seconds())
        self.assertEqual(test_cd.timeout_ms, 100)
        self.assertTrue(test_cd.busy())
        self.assertFalse(test_cd.timed_out())
        time.sleep(0.1)
        self.assertTrue(test_cd.timed_out())
        test_cd.reset(timedelta(seconds=0.5))
        self.assertTrue(test_cd.remaining_time().total_seconds() * 1000 > 45)
        self.assertTrue(test_cd.busy())
        self.assertFalse(test_cd.timed_out())
        test_cd.reset(timedelta(milliseconds=tm))
        self.assertTrue(test_cd.busy())
        test_cd.time_out()
        self.assertTrue(test_cd.timed_out())

    def test_tm_header(self):
        sp_header = SpacePacketHeader.tm(apid=0x03, data_len=16, seq_count=35)
        self.assertEqual(sp_header.apid, 0x03)
        self.assertEqual(sp_header.seq_flags, SequenceFlags.UNSEGMENTED)
        self.assertEqual(sp_header.ccsds_version, 0b000)
        self.assertEqual(sp_header.packet_id, PacketId(PacketType.TM, False, 0x03))
        self.assertEqual(
            sp_header.packet_seq_control,
            PacketSeqCtrl(SequenceFlags.UNSEGMENTED, 35),
        )
        self.assertEqual(sp_header.seq_count, 35)
        self.assertEqual(sp_header.data_len, 16)
        self.assertEqual(sp_header.packet_type, PacketType.TM)

    def test_len_field_setter(self):
        self.sp_header.set_data_len_from_packet_len(10)
        # Total packet length minus the header lenght minus 1
        self.assertEqual(self.sp_header.data_len, 3)

    def test_invalid_len_field_setter_call(self):
        for idx in range(7):
            with self.assertRaises(ValueError):
                self.sp_header.set_data_len_from_packet_len(idx)

    def test_tc_header(self):
        sp_header = SpacePacketHeader.tc(apid=0x7FF, data_len=16, seq_count=0x3FFF)
        self.assertEqual(sp_header.apid, 0x7FF)
        self.assertEqual(sp_header.seq_flags, SequenceFlags.UNSEGMENTED)
        self.assertEqual(sp_header.ccsds_version, 0b000)
        self.assertEqual(sp_header.packet_id, PacketId(PacketType.TC, False, 0x7FF))
        self.assertEqual(
            sp_header.packet_seq_control,
            PacketSeqCtrl(SequenceFlags.UNSEGMENTED, 0x3FFF),
        )
        self.assertEqual(sp_header.seq_count, 0x3FFF)
        self.assertEqual(sp_header.data_len, 16)
        self.assertEqual(sp_header.packet_type, PacketType.TC)

    def test_raw_output(self):
        raw_output = self.sp_header.pack()
        self.assertEqual(
            raw_output,
            bytes(
                [
                    0x18,  # TC, and secondary header flag is set -> 0b0001100 -> 0x18
                    0x02,  # APID 0x02
                    0x40,  # Sequence count is one byte value, so the only set bit here is the bit
                    # from the Sequence flag argument, which is the second bit for
                    # SequenceFlags.FIRST_SEGMENT
                    0x34,  # Sequence Count specified above
                    0x00,  # This byte and the next byte should be 22 big endian (packet length)
                    0x16,
                ]
            ),
        )

    def test_more_complex_output(self):
        # All ones, maximum value for APID
        self.sp_header.apid = pow(2, 11) - 1
        # All ones, maximum value for sequence count
        self.sp_header.seq_count = pow(2, 14) - 1
        self.sp_header.seq_flags = SequenceFlags.UNSEGMENTED
        self.sp_header.data_len = pow(2, 16) - 1
        raw_output = self.sp_header.pack()
        self.assertEqual(
            raw_output,
            bytes(
                [
                    0x1F,  # APID part is all ones, TC, sec header flag set -> 0b00011111
                    0xFF,
                    0xFF,  # All-Ones PSC
                    0xFF,
                    0xFF,  # This byte and the next byte should be 22 big endian (packet length)
                    0xFF,
                ]
            ),
        )

    def test_repr(self):
        self.assertEqual(
            f"{self.sp_header!r}",
            f"SpacePacketHeader(packet_version=0, packet_type={PacketType.TC!r}, "
            f"apid={self.sp_header.apid}, seq_cnt={self.sp_header.seq_count}, "
            f"data_len={self.sp_header.data_len}, "
            f"sec_header_flag={self.sp_header.sec_header_flag}, "
            f"seq_flags={self.sp_header.seq_flags!r})",
        )

    def test_apid_from_raw(self):
        sp_packed = self.sp_header.pack()
        self.assertEqual(get_apid_from_raw_space_packet(raw_packet=sp_packed), 0x02)

    def test_apid_from_raw_invalid_input(self):
        with self.assertRaises(ValueError):
            get_apid_from_raw_space_packet(raw_packet=b"")

    def test_unpack(self):
        sp_packed = self.sp_header.pack()
        sp_unpacked = SpacePacketHeader.unpack(data=sp_packed)
        self.assertEqual(sp_unpacked.packet_type, PacketType.TC)
        self.assertEqual(sp_unpacked.apid, 0x02)
        self.assertEqual(sp_unpacked.ccsds_version, 0b000)
        self.assertEqual(sp_unpacked.seq_count, 52)
        self.assertEqual(sp_unpacked.seq_flags, SequenceFlags.FIRST_SEGMENT)

    def test_invalid_apid(self):
        with self.assertRaises(ValueError):
            SpacePacketHeader(apid=982292, data_len=22, seq_count=52, packet_type=PacketType.TC)

    def test_invalid_data_len(self):
        self.assertRaises(
            ValueError,
            SpacePacketHeader,
            apid=0x02,
            data_len=679393,
            seq_count=52,
            packet_type=PacketType.TC,
        )

    def test_invalid_seq_count(self):
        self.assertRaises(
            ValueError,
            SpacePacketHeader,
            apid=0x02,
            data_len=22,
            seq_count=96030,
            packet_type=PacketType.TC,
        )

    def test_unpack_invalid_input(self):
        self.assertRaises(ValueError, SpacePacketHeader.unpack, bytearray())

    def test_print(self):
        print(self.sp_header)
        print(self.sp_header.__repr__())

    def test_sp_packet_id_bytes(self):
        byte_one, byte_two = get_space_packet_id_bytes(
            packet_type=PacketType.TC, apid=0x3FF, secondary_header_flag=True
        )
        self.assertEqual(byte_two, 0xFF)
        self.assertEqual(byte_one & 0x07, 0x03)

    def test_packet_id(self):
        byte_one, byte_two = get_space_packet_id_bytes(
            packet_type=PacketType.TC, apid=0x3FF, secondary_header_flag=True
        )
        packet_id_as_num = byte_one << 8 | byte_two
        packet_id = PacketId(ptype=PacketType.TC, apid=0x3FF, sec_header_flag=True)
        packet_id_raw = get_sp_packet_id_raw(
            packet_type=PacketType.TC, apid=0x3FF, secondary_header_flag=True
        )
        self.assertEqual(packet_id_as_num, packet_id.raw())
        self.assertEqual(packet_id_as_num, packet_id_raw)
        self.assertFalse(
            packet_id == PacketSeqCtrl(seq_flags=SequenceFlags.UNSEGMENTED, seq_count=0x22)
        )
        test_dict = {packet_id: 1}
        self.assertEqual(test_dict[packet_id], 1)
        test_dict.update({packet_id: 2})
        self.assertEqual(test_dict[packet_id], 2)

    def test_packet_seq_ctrl(self):
        psc = PacketSeqCtrl(seq_count=0x22, seq_flags=SequenceFlags.UNSEGMENTED)
        psc_raw = get_sp_psc_raw(seq_count=0x22, seq_flags=SequenceFlags.UNSEGMENTED)
        self.assertEqual(psc_raw, psc.raw())
        sequence_flags_raw = psc.seq_flags
        ssc_raw = psc.seq_count
        self.assertEqual(sequence_flags_raw, SequenceFlags.UNSEGMENTED)
        self.assertEqual(ssc_raw, 0x22)
        self.assertRaises(
            ValueError,
            get_sp_psc_raw,
            seq_count=0xFFFF,
            seq_flags=SequenceFlags.UNSEGMENTED,
        )
        self.assertFalse(psc == PacketId(ptype=PacketType.TC, apid=0x3FF, sec_header_flag=True))
        test_dict = {psc: 1}
        self.assertEqual(test_dict[psc], 1)
        test_dict.update({psc: 2})
        self.assertEqual(test_dict[psc], 2)

    def test_from_composite_field(self):
        packet_id = PacketId(ptype=PacketType.TC, apid=0x3FF, sec_header_flag=True)
        psc = PacketSeqCtrl(seq_count=0x22, seq_flags=SequenceFlags.UNSEGMENTED)
        raw_header = SpacePacketHeader.from_composite_fields(
            packet_id=packet_id, psc=psc, data_length=22
        ).pack()
        self.assertEqual(raw_header[0], ((packet_id.raw() & 0xFF00) >> 8) & 0x1FFF)
        self.assertEqual(raw_header[1], packet_id.raw() & 0xFF)
        self.assertEqual(raw_header[2], (psc.raw() & 0xFF00) >> 8)
        self.assertEqual(raw_header[3], psc.raw() & 0xFF)
        self.assertEqual(raw_header[4], (22 & 0xFF00) >> 8)
        self.assertEqual(raw_header[5], 22 & 0xFF)

        header_from_composite = SpacePacketHeader.from_composite_fields(
            packet_id=packet_id, psc=psc, data_length=22
        )
        self.assertEqual(header_from_composite.pack(), raw_header)
        header_tm = SpacePacketHeader(
            packet_type=PacketType.TM,
            seq_flags=SequenceFlags.UNSEGMENTED,
            apid=0x12,
            data_len=7,
            seq_count=28,
        )
        raw = header_tm.pack()
        header_tm_back = SpacePacketHeader.unpack(raw)
        self.assertEqual(header_tm_back.packet_type, PacketType.TM)
        self.assertEqual(header_tm_back.apid, 0x12)
        self.assertEqual(header_tm_back.ccsds_version, 0b000)
        self.assertEqual(header_tm_back.seq_count, 28)
        self.assertEqual(header_tm_back.data_len, 7)

    def test_to_space_packet(self):
        sph = SpacePacketHeader(
            PacketType.TC,
            apid=0x22,
            sec_header_flag=False,
            seq_flags=SequenceFlags.UNSEGMENTED,
            data_len=65,
            seq_count=22,
        )
        self.assertEqual(sph.header_len, 6)
        # User data mandatory
        with self.assertRaises(ValueError):
            SpacePacket(sp_header=sph, sec_header=None, user_data=None).pack()
        sph.sec_header_flag = True
        # Secondary header mandatory
        with self.assertRaises(ValueError):
            SpacePacket(sp_header=sph, sec_header=None, user_data=None).pack()
        sph.packet_type = PacketType.TM
        self.assertEqual(sph.packet_type, PacketType.TM)

    def test_sp_print(self):
        sph = SpacePacketHeader(
            PacketType.TC,
            apid=0x22,
            sec_header_flag=False,
            seq_flags=SequenceFlags.UNSEGMENTED,
            data_len=65,
            seq_count=22,
        )
        sp = SpacePacket(sp_header=sph, user_data=bytes([0, 1]), sec_header=None)
        print(sp)

    def test_utility(self):
        psc = PacketSeqCtrl(seq_flags=SequenceFlags.UNSEGMENTED, seq_count=pow(2, 14) - 1)
        self.assertEqual(
            f"{psc}",
            f"PSC: [Seq Flags: UNSEG, Seq Count: {pow(2, 14) - 1}]",
        )
        psc_raw = psc.raw()
        self.assertEqual(psc_raw, 0xFFFF)
        psc_from_raw = PacketSeqCtrl.from_raw(psc_raw)
        self.assertEqual(psc_from_raw.raw(), psc.raw())
        self.assertEqual(PacketSeqCtrl.empty().raw(), 0)

        packet_id = PacketId(ptype=PacketType.TC, sec_header_flag=True, apid=0x7FF)
        self.assertEqual(
            f"{packet_id}",
            "Packet ID: [Packet Type: TC, Sec Header Flag: True, APID: 0x7ff]",
        )
        packet_id_raw = packet_id.raw()
        self.assertEqual(packet_id_raw, 0x1FFF)
        packet_id_from_raw = PacketId.from_raw(packet_id_raw)
        self.assertEqual(packet_id_from_raw.raw(), packet_id.raw())
        self.assertEqual(PacketId.empty().raw(), 0)

    def test_equality_sp_packet(self):
        sp = SpacePacket(sp_header=self.sp_header, sec_header=None, user_data=bytes([0, 1, 2]))
        other_sp = SpacePacket(
            sp_header=self.sp_header, sec_header=None, user_data=bytes([0, 1, 2])
        )
        self.assertEqual(sp, other_sp)

import fastcrc

from spacepackets.uslp.defs import (
    UslpFhpVhopFieldMissingError,
    UslpInvalidConstructionRulesError,
    UslpInvalidFrameHeaderError,
    UslpTruncatedFrameNotAllowedError,
)
from spacepackets.uslp.frame import (
    FixedFrameProperties,
    FrameType,
    TfdzConstructionRules,
    TransferFrame,
    TransferFrameDataField,
    UslpProtocolIdentifier,
    VarFrameProperties,
)
from spacepackets.uslp.header import (
    BypassSequenceControlFlag,
    HeaderType,
    PrimaryHeader,
    ProtocolCommandFlag,
    SourceOrDestField,
    TruncatedPrimaryHeader,
    UslpInvalidRawPacketOrFrameLenError,
    UslpTypeMissmatchError,
    UslpVersionMissmatchError,
    determine_header_type,
)


class TestUslp(TestCase):
    def setUp(self) -> None:
        # Initialize with frame length 0 and set frame length field later with a helper function
        self.primary_header = PrimaryHeader(
            scid=0x10,
            map_id=0b0011,
            src_dest=SourceOrDestField.SOURCE,
            vcid=0b110111,
            frame_len=0,
            op_ctrl_flag=False,
            vcf_count_len=0,
            prot_ctrl_cmd_flag=ProtocolCommandFlag.USER_DATA,
            bypass_seq_ctrl_flag=BypassSequenceControlFlag.SEQ_CTRLD_QOS,
        )
        self.fp_tfdf = TransferFrameDataField(
            tfdz_cnstr_rules=TfdzConstructionRules.FpPacketSpanningMultipleFrames,
            uslp_ident=UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
            tfdz=bytearray(),
            fhp_or_lvop=0,
        )
        # Packet without op control field, without insert zone and without frame error control
        self.transfer_frame = TransferFrame(
            header=self.primary_header,
            op_ctrl_field=None,
            insert_zone=None,
            tfdf=self.fp_tfdf,
            has_fecf=False,
        )
        # This sets the correct frame length in the primary header
        self.transfer_frame.set_frame_len_in_header()
        return super().setUp()

    def test_header(self):
        primary_header = PrimaryHeader(
            scid=pow(2, 16) - 2,
            map_id=0b0011,
            src_dest=SourceOrDestField.SOURCE,
            vcid=0b110111,
            frame_len=pow(2, 16) - 3,
            op_ctrl_flag=True,
            vcf_count_len=0,
            prot_ctrl_cmd_flag=ProtocolCommandFlag.PROTOCOL_INFORMATION,
            bypass_seq_ctrl_flag=BypassSequenceControlFlag.EXPEDITED_QOS,
        )
        self.assertEqual(primary_header.truncated(), False)
        self.assertEqual(primary_header.len(), 7)
        packed_header = primary_header.pack()
        self.assertEqual((packed_header[0] >> 4) & 0b1111, 0x0C)
        # First four bits of SCID should be all ones
        self.assertEqual((packed_header[0] & 0x0F), 0b1111)
        # Next eight bits should be all ones
        self.assertEqual(packed_header[1], 0xFF)
        # Last four bits should be 0b1110
        self.assertEqual((packed_header[2] >> 4) & 0b1111, 0b1110)
        # Source or destination ID, should be 0
        self.assertEqual((packed_header[2] >> 3) & 0x01, 0)
        # The next three bits are the first three bits of the virtual channel
        self.assertEqual(packed_header[2] & 0b111, 0b110)
        # The first three bits of the next byte are the last three bits of the virtual channel
        self.assertEqual((packed_header[3] >> 5) & 0b111, 0b111)
        # The next four bits are the map ID
        self.assertEqual((packed_header[3] >> 1) & 0b1111, 0b0011)
        # End of frame primary header. Should be 0 for non-trucated frame
        self.assertEqual(packed_header[3] & 0x01, 0)
        # Frame length is 0xfffd
        self.assertEqual(packed_header[4], 0xFF)
        self.assertEqual(packed_header[5], 0xFD)
        # Bypass / Sequence Control is 1
        self.assertEqual((packed_header[6] >> 7) & 0x01, 1)
        # Protocol Control Command Flag is 1
        self.assertEqual((packed_header[6] >> 6) & 0x01, 1)
        # Spares are 0
        self.assertEqual((packed_header[6] >> 4) & 1, 0b00)
        # OCF flag is 1
        self.assertEqual((packed_header[6] >> 3) & 1, 1)
        # VCF frame count is 0
        self.assertEqual(packed_header[6] & 0b111, 0)
        self.assertEqual(determine_header_type(packed_header), HeaderType.NON_TRUNCATED)

        primary_header.vcf_count_len = 1
        primary_header.vcf_count = 0xAF
        header_with_vcf_count = primary_header.pack()
        self.assertEqual(header_with_vcf_count[6] & 0b111, 0x01)
        self.assertEqual(header_with_vcf_count[7], 0xAF)
        primary_header.vcf_count_len = 2
        primary_header.vcf_count = 0xAFFE
        unpacked_vcf_1 = PrimaryHeader.unpack(raw_packet=header_with_vcf_count)
        self.assertEqual(unpacked_vcf_1.vcf_count_len, 1)
        self.assertEqual(unpacked_vcf_1.vcf_count, 0xAF)

        header_with_vcf_count = primary_header.pack()
        self.assertEqual(header_with_vcf_count[6] & 0b111, 2)
        self.assertEqual(header_with_vcf_count[7], 0xAF)
        self.assertEqual(header_with_vcf_count[8], 0xFE)
        unpacked_vcf_2 = PrimaryHeader.unpack(raw_packet=header_with_vcf_count)
        self.assertEqual(unpacked_vcf_2.vcf_count_len, 2)
        self.assertEqual(unpacked_vcf_2.vcf_count, 0xAFFE)

        primary_header.vcf_count_len = 3
        primary_header.vcf_count = 0xAFFEFE
        header_with_vcf_count = primary_header.pack()
        self.assertEqual(header_with_vcf_count[6] & 0b111, 3)
        self.assertEqual(header_with_vcf_count[7], 0xAF)
        self.assertEqual(header_with_vcf_count[8], 0xFE)
        self.assertEqual(header_with_vcf_count[9], 0xFE)
        unpacked_vcf_3 = PrimaryHeader.unpack(raw_packet=header_with_vcf_count)
        self.assertEqual(unpacked_vcf_3.vcf_count_len, 3)
        self.assertEqual(unpacked_vcf_3.vcf_count, 0xAFFEFE)

        primary_header.vcf_count_len = 4
        primary_header.vcf_count = 0xAFFECAFE
        header_with_vcf_count = primary_header.pack()
        self.assertEqual(header_with_vcf_count[6] & 0b111, 4)
        self.assertEqual(header_with_vcf_count[7], 0xAF)
        self.assertEqual(header_with_vcf_count[8], 0xFE)
        self.assertEqual(header_with_vcf_count[9], 0xCA)
        self.assertEqual(header_with_vcf_count[10], 0xFE)
        unpacked_vcf_4 = PrimaryHeader.unpack(raw_packet=header_with_vcf_count)
        self.assertEqual(unpacked_vcf_4.vcf_count_len, 4)
        self.assertEqual(unpacked_vcf_4.vcf_count, 0xAFFECAFE)

        primary_header.vcf_count_len = 7
        primary_header.vcf_count = 0xAFFECAFEBABEAF
        header_with_vcf_count = primary_header.pack()
        self.assertEqual(header_with_vcf_count[6] & 0b111, 7)
        self.assertEqual(header_with_vcf_count[7], 0xAF)
        self.assertEqual(header_with_vcf_count[8], 0xFE)
        self.assertEqual(header_with_vcf_count[9], 0xCA)
        self.assertEqual(header_with_vcf_count[10], 0xFE)
        self.assertEqual(header_with_vcf_count[11], 0xBA)
        self.assertEqual(header_with_vcf_count[12], 0xBE)
        self.assertEqual(header_with_vcf_count[13], 0xAF)
        unpacked_with_vcf = PrimaryHeader.unpack(raw_packet=header_with_vcf_count)
        self.assertEqual(unpacked_with_vcf.vcf_count_len, 7)
        self.assertEqual(unpacked_with_vcf.vcf_count, 0xAFFECAFEBABEAF)
        unpacked_primary_header = PrimaryHeader.unpack(raw_packet=packed_header)
        # Check field validity by serializing unpacked header again
        self.assertEqual(packed_header, unpacked_primary_header.pack())
        self.assertRaises(UslpTypeMissmatchError, TruncatedPrimaryHeader.unpack, packed_header)
        self.assertRaises(
            UslpInvalidRawPacketOrFrameLenError,
            TruncatedPrimaryHeader.unpack,
            bytearray(),
        )
        self.assertRaises(
            UslpInvalidRawPacketOrFrameLenError,
            PrimaryHeader.unpack,
            header_with_vcf_count[0:7],
        )
        crap_with_valid_parsing_fields = bytearray(5)
        crap_with_valid_parsing_fields[0] = 0b11000000
        self.assertRaises(
            UslpInvalidRawPacketOrFrameLenError,
            PrimaryHeader.unpack,
            crap_with_valid_parsing_fields,
        )
        # Set invalid version
        crap_with_valid_parsing_fields[0] = 0b0001000
        crap_with_valid_parsing_fields.extend(bytearray(3))
        self.assertRaises(
            UslpVersionMissmatchError,
            PrimaryHeader.unpack,
            crap_with_valid_parsing_fields,
        )
        truncated_header = TruncatedPrimaryHeader(
            scid=0b0001000100010001,
            map_id=0b1101,
            vcid=0b101101,
            src_dest=SourceOrDestField.DEST,
        )
        self.assertEqual(truncated_header.truncated(), True)
        self.assertEqual(truncated_header.len(), 4)
        packed_header = truncated_header.pack()
        self.assertEqual((packed_header[0] >> 4) & 0b1111, 0x0C)
        self.assertEqual((packed_header[0] & 0x0F), 0b0001)
        self.assertEqual(packed_header[1], 0b00010001)
        self.assertEqual((packed_header[2] >> 4) & 0b1111, 0b0001)
        # source or dest ID is 1
        self.assertEqual((packed_header[2] >> 3) & 0x01, 1)
        # The next three bits are the first three bits of the virtual channel
        self.assertEqual(packed_header[2] & 0b111, 0b101)
        # The first three bits of the next byte are the last three bits of the virtual channel
        self.assertEqual((packed_header[3] >> 5) & 0b111, 0b101)
        # The next four bits are the map ID
        self.assertEqual((packed_header[3] >> 1) & 0b1111, 0b1101)
        # End of frame primary header. Should be 1 for truncated frame
        self.assertEqual(packed_header[3] & 0x01, 1)
        self.assertEqual(determine_header_type(packed_header), HeaderType.TRUNCATED)

        self.assertRaises(ValueError, determine_header_type, bytearray())
        tmp = truncated_header.vcid
        truncated_header.vcid = 0xFFF
        self.assertRaises(ValueError, truncated_header.pack)
        truncated_header.vcid = tmp
        tmp = truncated_header.scid
        truncated_header.scid = 0xFFFFF
        self.assertRaises(ValueError, truncated_header.pack)
        truncated_header.scid = tmp
        tmp = truncated_header.map_id
        truncated_header.map_id = 0xFFFFF
        self.assertRaises(ValueError, truncated_header.pack)
        truncated_header.map_id = tmp
        unpacked_truncated = TruncatedPrimaryHeader.unpack(raw_packet=packed_header)
        self.assertEqual(unpacked_truncated.pack(), packed_header)

    def test_invalid_header(self):
        with self.assertRaises(ValueError):
            PrimaryHeader(
                scid=pow(2, 16),
                map_id=0b0011,
                src_dest=SourceOrDestField.SOURCE,
                vcid=0b110111,
                frame_len=100,
                op_ctrl_flag=True,
                vcf_count_len=0,
                vcf_count=255,
                prot_ctrl_cmd_flag=ProtocolCommandFlag.PROTOCOL_INFORMATION,
                bypass_seq_ctrl_flag=BypassSequenceControlFlag.EXPEDITED_QOS,
            )

    def test_frame_pack(self):
        # This sets the correct frame length in the primary header
        self.transfer_frame.set_frame_len_in_header()
        frame_packed = self.transfer_frame.pack(truncated=False, frame_type=FrameType.FIXED)
        # 7 byte primary header + TFDF with 3 bytes, TFDZ is empty
        self.assertEqual(len(frame_packed), 10)
        self.assertEqual((frame_packed[4] << 8) | frame_packed[5], len(frame_packed) - 1)
        # The primary header was already unit-tested, its fields won't be checked again
        self.assertEqual(frame_packed[7], 0x00)
        self.assertEqual(frame_packed[8], 0x00)
        self.assertEqual(frame_packed[9], 0x00)
        self.transfer_frame.tfdf.tfdz_contr_rules = TfdzConstructionRules.FpFixedStartOfMapaSDU
        self.transfer_frame.tfdf.uslp_ident = UslpProtocolIdentifier.IDLE_DATA
        self.transfer_frame.tfdf.fhp_or_lvop = 0xAFFE
        frame_packed = self.transfer_frame.pack()
        self.assertEqual(len(frame_packed), 10)
        self.assertEqual((frame_packed[4] << 8) | frame_packed[5], len(frame_packed) - 1)
        # TFDZ construction rule is 0b001, USLP identifier is 0b11111, concatenation is 0x3f
        self.assertEqual(frame_packed[7], 0x3F)
        # This pointer does not really make sense for an empty TFDZ, but we just check that is was
        # still set correctly
        self.assertEqual(frame_packed[8], 0xAF)
        self.assertEqual(frame_packed[9], 0xFE)

    def test_frame_unpack(self):
        frame_properties = FixedFrameProperties(has_insert_zone=False, has_fecf=False, fixed_len=10)
        self.fp_tfdf.uslp_ident = UslpProtocolIdentifier.IDLE_DATA
        self.fp_tfdf.tfdz_contr_rules = TfdzConstructionRules.FpFixedStartOfMapaSDU
        self.fp_tfdf.fhp_or_lvop = 0xAFFE
        frame_packed = self.transfer_frame.pack(truncated=False, frame_type=FrameType.FIXED)
        frame_unpacked = TransferFrame.unpack(
            raw_frame=frame_packed,
            frame_type=FrameType.FIXED,
            frame_properties=frame_properties,
        )
        self.assertEqual(frame_unpacked.insert_zone, None)
        self.assertEqual(frame_unpacked.has_fecf, False)
        self.assertEqual(frame_unpacked.op_ctrl_field, None)
        self.assertEqual(frame_unpacked.tfdf.uslp_ident, UslpProtocolIdentifier.IDLE_DATA)
        self.assertEqual(
            frame_unpacked.tfdf.tfdz_contr_rules,
            TfdzConstructionRules.FpFixedStartOfMapaSDU,
        )
        self.assertEqual(frame_unpacked.tfdf.fhp_or_lvop, 0xAFFE)
        self.assertEqual(frame_unpacked.tfdf.tfdz, bytearray())
        self.assertEqual(frame_unpacked.header.pack(), self.transfer_frame.header.pack())

    def test_some_errors(self):
        with self.assertRaises(ValueError):
            FixedFrameProperties(fixed_len=5, has_fecf=False, has_insert_zone=True)
        with self.assertRaises(ValueError):
            invalid_tfdz = bytearray(70000)
            tfdf = TransferFrameDataField(
                tfdz_cnstr_rules=TfdzConstructionRules.FpPacketSpanningMultipleFrames,
                uslp_ident=UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
                tfdz=invalid_tfdz,
                fhp_or_lvop=0,
            )
        self.assertEqual(self.fp_tfdf.verify_frame_type(frame_type=FrameType.FIXED), True)
        self.assertEqual(self.fp_tfdf.verify_frame_type(frame_type=FrameType.VARIABLE), False)
        vp_tfdf = TransferFrameDataField(
            tfdz_cnstr_rules=TfdzConstructionRules.VpNoSegmentation,
            uslp_ident=UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
            tfdz=bytearray(),
        )
        self.assertEqual(vp_tfdf.verify_frame_type(frame_type=FrameType.VARIABLE), True)
        self.assertEqual(
            vp_tfdf.should_have_fhp_or_lvp_field(truncated=False, frame_type=FrameType.VARIABLE),
            False,
        )
        self.assertEqual(
            self.fp_tfdf.should_have_fhp_or_lvp_field(truncated=False, frame_type=FrameType.FIXED),
            True,
        )
        empty_short_tfdf = TransferFrameDataField(
            tfdz_cnstr_rules=TfdzConstructionRules.FpPacketSpanningMultipleFrames,
            uslp_ident=UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
            fhp_or_lvop=0,
            tfdz=bytearray(),
        )
        packed_short_tfdf = empty_short_tfdf.pack(truncated=False, frame_type=FrameType.FIXED)
        self.assertEqual(len(packed_short_tfdf), 3)
        TransferFrameDataField.unpack(
            raw_tfdf=packed_short_tfdf,
            frame_type=FrameType.FIXED,
            truncated=False,
            exact_len=3,
        )

        tfdf_vp = TransferFrameDataField(
            tfdz_cnstr_rules=TfdzConstructionRules.VpLastSegment,
            uslp_ident=UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
            tfdz=bytearray(),
        )
        tfdf_vp_packed = tfdf_vp.pack()
        with self.assertRaises(UslpFhpVhopFieldMissingError):
            TransferFrameDataField(
                tfdz_cnstr_rules=TfdzConstructionRules.FpFixedStartOfMapaSDU,
                uslp_ident=UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
                tfdz=bytearray(),
            ).pack()
        with self.assertRaises(UslpInvalidRawPacketOrFrameLenError):
            TransferFrameDataField.unpack(
                raw_tfdf=bytearray(),
                truncated=False,
                exact_len=0,
                frame_type=FrameType.FIXED,
            )
        with self.assertRaises(UslpInvalidConstructionRulesError):
            short_tfdf = empty_short_tfdf.pack()
            short_tfdf[0] &= ~0b11100000
            short_tfdf[0] |= TfdzConstructionRules.VpNoSegmentation << 5
            TransferFrameDataField.unpack(
                raw_tfdf=short_tfdf,
                frame_type=FrameType.FIXED,
                exact_len=1,
                truncated=False,
            )
        tfdf_vp_unpacked = TransferFrameDataField.unpack(
            raw_tfdf=tfdf_vp_packed,
            frame_type=FrameType.VARIABLE,
            exact_len=len(tfdf_vp_packed),
            truncated=False,
        )
        self.assertEqual(tfdf_vp_unpacked.tfdz, bytearray())
        self.assertEqual(
            tfdf_vp_unpacked.uslp_ident,
            UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
        )
        self.assertEqual(tfdf_vp_unpacked.tfdz_contr_rules, TfdzConstructionRules.VpLastSegment)
        primary_header = PrimaryHeader(
            scid=0x10,
            map_id=0b0011,
            src_dest=SourceOrDestField.SOURCE,
            vcid=0b110111,
            frame_len=0,
            op_ctrl_flag=True,
            vcf_count_len=0,
            prot_ctrl_cmd_flag=ProtocolCommandFlag.USER_DATA,
            bypass_seq_ctrl_flag=BypassSequenceControlFlag.SEQ_CTRLD_QOS,
        )
        tfdf = TransferFrameDataField(
            tfdz_cnstr_rules=TfdzConstructionRules.FpPacketSpanningMultipleFrames,
            uslp_ident=UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
            tfdz=bytearray([1, 2, 3, 4]),
            fhp_or_lvop=0,
        )
        insert_zone = bytearray(4)
        op_ctrl_field = bytearray([4, 3, 2, 1])
        larger_frame = TransferFrame(
            header=primary_header,
            insert_zone=insert_zone,
            op_ctrl_field=op_ctrl_field,
            tfdf=tfdf,
            has_fecf=True,
        )
        larger_frame.set_frame_len_in_header()
        self.assertEqual(larger_frame.header.frame_len, 24 - 1)
        self.assertEqual(larger_frame.len(), 24)
        larger_frame_packed = larger_frame.pack()
        self.assertEqual(len(larger_frame_packed), 24)
        self.assertEqual(larger_frame_packed[7 : 7 + 4], bytearray(4))
        self.assertEqual(larger_frame_packed[14 : 14 + 4], bytearray([1, 2, 3, 4]))
        self.assertEqual(larger_frame_packed[18 : 18 + 4], bytearray([4, 3, 2, 1]))
        crc16 = struct.pack("!H", fastcrc.crc16.ibm_3740(bytes(larger_frame_packed[0:22])))
        self.assertEqual(larger_frame_packed[22:], crc16)
        with self.assertRaises(UslpInvalidRawPacketOrFrameLenError):
            TransferFrame.unpack(
                raw_frame=larger_frame_packed,
                frame_type=FrameType.FIXED,
                frame_properties=FixedFrameProperties(
                    fixed_len=28,
                    has_insert_zone=True,
                    insert_zone_len=4,
                    has_fecf=True,
                ),
            )
        with self.assertRaises(UslpInvalidRawPacketOrFrameLenError):
            TransferFrame.unpack(
                raw_frame=larger_frame_packed,
                frame_type=FrameType.FIXED,
                frame_properties=FixedFrameProperties(
                    fixed_len=22,
                    has_insert_zone=True,
                    insert_zone_len=4,
                    has_fecf=True,
                ),
            )
        with self.assertRaises(ValueError):
            TransferFrame.unpack(
                raw_frame=larger_frame_packed,
                frame_type=FrameType.FIXED,
                frame_properties=VarFrameProperties(
                    truncated_frame_len=12,
                    has_insert_zone=True,
                    insert_zone_len=4,
                    has_fecf=True,
                ),
            )
        larger_frame.header.op_ctrl_flag = False
        with self.assertRaises(UslpInvalidFrameHeaderError):
            larger_frame.pack()
        larger_frame.header.op_ctrl_flag = True
        larger_frame.op_ctrl_field = None
        with self.assertRaises(UslpInvalidFrameHeaderError):
            larger_frame.pack()
        larger_frame.header.op_ctrl_flag = True
        # Invalid length
        larger_frame.op_ctrl_field = bytearray([0, 0, 0])
        with self.assertRaises(ValueError):
            larger_frame.pack()
        larger_frame_unpacked = TransferFrame.unpack(
            raw_frame=larger_frame_packed,
            frame_type=FrameType.FIXED,
            frame_properties=FixedFrameProperties(
                fixed_len=24,
                has_insert_zone=True,
                insert_zone_len=4,
                has_fecf=True,
            ),
        )
        self.assertEqual(larger_frame_unpacked.insert_zone, bytearray(4))
        self.assertEqual(larger_frame_unpacked.op_ctrl_field, bytearray([4, 3, 2, 1]))
        with self.assertRaises(UslpInvalidRawPacketOrFrameLenError):
            TransferFrame.unpack(
                raw_frame=bytearray(3),
                frame_type=FrameType.FIXED,
                frame_properties=FixedFrameProperties(
                    fixed_len=24,
                    has_insert_zone=True,
                    insert_zone_len=4,
                    has_fecf=True,
                ),
            )
        truncated_header = TruncatedPrimaryHeader(
            scid=12, src_dest=SourceOrDestField.SOURCE, map_id=12, vcid=5
        )
        tfdf_truncated = TransferFrameDataField(
            tfdz_cnstr_rules=TfdzConstructionRules.VpNoSegmentation,
            uslp_ident=UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
            tfdz=bytearray([1, 2, 3, 4]),
        )
        truncated_frame = TransferFrame(
            header=truncated_header,
            insert_zone=None,
            op_ctrl_field=None,
            tfdf=tfdf_truncated,
            has_fecf=False,
        )
        truncated_frame_packed = truncated_frame.pack(truncated=True)
        self.assertEqual(len(truncated_frame_packed), 9)
        with self.assertRaises(UslpTruncatedFrameNotAllowedError):
            TransferFrame.unpack(
                raw_frame=truncated_frame_packed,
                frame_type=FrameType.FIXED,
                frame_properties=FixedFrameProperties(
                    fixed_len=len(truncated_frame_packed),
                    has_insert_zone=True,
                    insert_zone_len=4,
                    has_fecf=True,
                ),
            )
        truncated_frame_unpacked = TransferFrame.unpack(
            raw_frame=truncated_frame_packed,
            frame_type=FrameType.VARIABLE,
            frame_properties=VarFrameProperties(
                truncated_frame_len=9, has_insert_zone=False, has_fecf=False
            ),
        )
        self.assertEqual(truncated_frame_unpacked.tfdf.fhp_or_lvop, None)
        self.assertEqual(
            truncated_frame_unpacked.tfdf.uslp_ident,
            UslpProtocolIdentifier.SPACE_PACKETS_ENCAPSULATION_PACKETS,
        )
        self.assertEqual(
            truncated_frame_unpacked.tfdf.tfdz_contr_rules,
            TfdzConstructionRules.VpNoSegmentation,
        )

if __name__ == '__main__':

    t = TestSpacePacket()
	t.test_basic_countdown()                       # test the coundown function
	t.setUp()
	t.test_basic()                                 # basic header set-update
	t.test_tm_header()
	t.test_len_field_setter()
	t.test_invalid_len_field_setter_call()
	t.test_tc_header()
	t.test_raw_output()
	t.test_more_complex_output()
	t.test_repr()
	t.test_apid_from_raw()
	t.test_apid_from_raw_invalid_input()
	t.test_unpack()
	t.test_invalid_apid()
	t.test_invalid_data_len()
	t.test_invalid_seq_count()
	t.test_unpack_invalid_input()
	t.test_sp_packet_id_bytes()
	t.test_packet_seq_ctrl()
	t.test_to_space_packet()
	t.test_from_composite_field()
	t.test_sp_print()
	t.test_utility()
    tt= TestUslp()                                          # uslp
    tt.setUp()
    tt.test_header()
    tt.test_frame_pack()
    tt.test_frame_unpack()    

 
