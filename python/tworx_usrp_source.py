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
import time

def gen_sig_io(num_elements):
    # Dynamically create types for signature
    io = []
    for i in range(num_elements):
        io.append(gr.sizeof_gr_complex*1)
    io.append(gr.sizeof_float*num_elements)
    return io

class tworx_usrp_source(gr.hier_block2):

    def __init__(self, samp_rate=1000000, center_freq=2400000000, gain=40, sources=4, addresses="addr=192.168.10.2", antenna="RX2"):
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
        self.sources_per_radio = int(sources / 2)
        self.addresses = addresses
        self.antenna = antenna

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0 = uhd.usrp_source(
                ",".join((self.addresses, "")),
                uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(self.sources),
                ),
        )

        subdevs = 'A:0 B:0'

        self.uhd_usrp_source_0.set_clock_source('external', 0)
        self.uhd_usrp_source_0.set_time_source('external', 0)
        self.uhd_usrp_source_0.set_clock_source('external', 1)
        self.uhd_usrp_source_0.set_time_source('external', 1)
        time.sleep(1)  # Let clocks settle
        self.uhd_usrp_source_0.set_time_unknown_pps(uhd.time_spec())
        time.sleep(1)  # Let clocks settle
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_subdev_spec(subdevs, 0)
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
        self.set_center_freq(center_freq,sources)

        ##################################################
        # Connections
        ##################################################
        for source in range(self.sources):
            self.connect((self.uhd_usrp_source_0, source), (self, source))

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

    def set_center_freq(self, center_freq, sources):
        # Tune all channels to the desired frequency
        tune_resp = self.uhd_usrp_source_0.set_center_freq(center_freq, 0)
        tune_req = uhd.tune_request(rf_freq=center_freq, rf_freq_policy=uhd.tune_request.POLICY_MANUAL,
               dsp_freq=tune_resp.actual_dsp_freq, dsp_freq_policy=uhd.tune_request.POLICY_MANUAL)

        self.uhd_usrp_source_0.set_center_freq(tune_req, 1)
        if sources==4:
            self.uhd_usrp_source_0.set_center_freq(tune_req, 2)
            self.uhd_usrp_source_0.set_center_freq(tune_req, 3)

        # Synchronize the tuned channels
        now = self.uhd_usrp_source_0.get_time_now()
        self.uhd_usrp_source_0.set_command_time(now + uhd.time_spec(0.1))

        self.uhd_usrp_source_0.set_center_freq(tune_req, 0)
        self.uhd_usrp_source_0.set_center_freq(tune_req, 1)
        if sources==4:
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
