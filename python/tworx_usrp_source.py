# -*- coding: utf-8 -*-
#
# Copyright 2016
# Travis F. Collins <travisfcollins@gmail.com>
# Srikanth Pagadarai <srikanth.pagadarai@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

from gnuradio import gr
from gnuradio import uhd
from gnuradio.filter import firdes
import pmt
import time

def gen_sig_io(num_elements):
    # Dynamically create types for signature
    io = []
    for i in range(num_elements):
        io.append(gr.sizeof_gr_complex*1)
    io.append(gr.sizeof_float*num_elements)
    return io

class hier_msg_handler(gr.sync_block):
    def __init__(self, msg_handler):
        gr.sync_block.__init__(self, "Msg Handler",
            in_sig=None,
            out_sig=None,
        )
        self.msg_port = "command"
        self.message_port_register_in(pmt.intern(self.msg_port))
        self.set_msg_handler(pmt.intern(self.msg_port), msg_handler)

class tworx_usrp_source(gr.hier_block2):

    def __init__(self, samp_rate=1000000, center_freq=2400000000, gain=40, sources=4, addresses="addr0=192.168.10.2, addr1=192.168.20.3", antenna="RX2", num_samps=1000):
        gr.hier_block2.__init__(
            self, "TwoRx USRP",
            gr.io_signature(0, 0, 0),
            gr.io_signaturev(sources, sources, gen_sig_io(sources)),
        )

        ##################################################
        # Parameters
        ##################################################
        self.samp_rate = samp_rate
        self.center_freq = center_freq
        self.gain = gain
        self.sources = sources
        self.addresses = addresses
        self.num_samps = num_samps
        if antenna == "Toggle":
            self.toggle = True
            self.antenna = "RX2"
        else:
            self.toggle = False
            self.antenna = antenna
        self.msg_port = "command"

        ##################################################
        # Blocks
        ##################################################
        if self.toggle:
            issue_stream_cmd_on_start=False
        else:
            issue_stream_cmd_on_start=True
        self.uhd_usrp_source_0 = uhd.usrp_source(
                ",".join((self.addresses, "")),
                uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(self.sources),
                ),
                issue_stream_cmd_on_start=issue_stream_cmd_on_start,
        )
        self.handle_messages = hier_msg_handler(self.msg_handler)

        self.message_port_register_hier_in("command")

        # Dumb hack for desk debugging TODO remove
        if self.sources == 4:
            clk_time_src = 'external'
            subdevs = 'A:0 B:0'
        else:
            clk_time_src = 'internal'
            subdevs = 'A:AB B:AB'

        self.uhd_usrp_source_0.set_clock_source(clk_time_src, 0)
        self.uhd_usrp_source_0.set_time_source(clk_time_src, 0)
        if self.sources == 4:
            self.uhd_usrp_source_0.set_clock_source(clk_time_src, 1)
            self.uhd_usrp_source_0.set_time_source(clk_time_src, 1)
        time.sleep(1)  # Let clocks settle
        self.uhd_usrp_source_0.set_time_unknown_pps(uhd.time_spec())
        time.sleep(1)  # Let clocks settle
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_subdev_spec(subdevs, 0)
        if self.sources == 4:
            self.uhd_usrp_source_0.set_subdev_spec(subdevs, 1)
        self.uhd_usrp_source_0.set_antenna(self.antenna, 0)
        self.uhd_usrp_source_0.set_antenna(self.antenna, 1)
        if self.sources == 4:
            self.uhd_usrp_source_0.set_antenna(self.antenna, 2)
            self.uhd_usrp_source_0.set_antenna(self.antenna, 3)


        # Set channel specific settings
        self.uhd_usrp_source_0.set_gain(gain, 0)
        self.uhd_usrp_source_0.set_auto_dc_offset(True, 0)
        self.uhd_usrp_source_0.set_gain(gain, 1)
        self.uhd_usrp_source_0.set_auto_dc_offset(True, 1)
        if self.sources == 4:
            self.uhd_usrp_source_0.set_gain(gain, 2)
            self.uhd_usrp_source_0.set_auto_dc_offset(True, 2)
            self.uhd_usrp_source_0.set_gain(gain, 3)
            self.uhd_usrp_source_0.set_auto_dc_offset(True, 3)

        # Use timed commands to set frequencies
        self.set_center_freq(center_freq)

        ##################################################
        # Connections
        ##################################################
        for source in range(self.sources):
            self.connect((self.uhd_usrp_source_0, source), (self, source))
        self.msg_connect((self, 'command'), (self.uhd_usrp_source_0, 'command'))
        self.msg_connect((self, 'command'), (self.handle_messages, 'command'))

        if not self.toggle:
            now = self.uhd_usrp_source_0.get_time_now()
            cmd = uhd.stream_cmd(uhd.stream_cmd.STREAM_MODE_START_CONTINUOUS)
            cmd.stream_now = False
            cmd.time_spec = now + uhd.time_spec(1)
            self.uhd_usrp_source_0.issue_stream_cmd(cmd)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.uhd_usrp_source_1.set_samp_rate(self.samp_rate)

    def set_center_freq(self, center_freq):
        # Tune all channels to the desired frequency
        tune_resp = self.uhd_usrp_source_0.set_center_freq(center_freq, 0)
        tune_req = uhd.tune_request(rf_freq=center_freq, rf_freq_policy=uhd.tune_request.POLICY_MANUAL,
               dsp_freq=tune_resp.actual_dsp_freq, dsp_freq_policy=uhd.tune_request.POLICY_MANUAL)

        self.uhd_usrp_source_0.set_center_freq(tune_req, 1)
        if self.sources==4:
            self.uhd_usrp_source_0.set_center_freq(tune_req, 2)
            self.uhd_usrp_source_0.set_center_freq(tune_req, 3)

        # Synchronize the tuned channels
        now = self.uhd_usrp_source_0.get_time_now()
        self.uhd_usrp_source_0.set_command_time(now + uhd.time_spec(0.1))

        self.uhd_usrp_source_0.set_center_freq(tune_req, 0)
        self.uhd_usrp_source_0.set_center_freq(tune_req, 1)
        if self.sources==4:
            self.uhd_usrp_source_0.set_center_freq(tune_req, 2)
            self.uhd_usrp_source_0.set_center_freq(tune_req, 3)

        self.uhd_usrp_source_0.clear_command_time()

    def get_center_freq(self):
        return self.center_freq


    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        print("DO NOT TUNE GAINS DURING RUNTIME")

    def get_sources(self):
        return self.sources

    def set_sources(self, sources):
        self.sources = sources

    def msg_handler(self, msg):
        if pmt.dict_has_key(msg, pmt.intern("antenna")): #and \
            #pmt.dict_has_key(msg, pmt.intern("time")):
            cmd = uhd.stream_cmd_t(uhd.stream_cmd_t.STREAM_MODE_NUM_SAMPS_AND_DONE)
            cmd.num_samps = self.num_samps
            # cmd.time_spec = pmt.dict_ref(msg, pmt.intern("time"), None)
            self.uhd_usrp_source_0.issue_stream_cmd(cmd)

