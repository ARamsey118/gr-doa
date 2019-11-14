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

from gnuradio import blocks
from gnuradio import gr
from gnuradio import uhd
from gnuradio.filter import firdes
import pmt
from threading import Thread
import time

def gen_sig_io(num_elements):
    # Dynamically create types for signature
    io = []
    for i in range(num_elements):
        io.append(gr.sizeof_gr_complex*1)
    io.append(gr.sizeof_float*num_elements)
    return io

class msg_strobe(gr.basic_block):
    def __init__(self, msgs, periods, add_time=False, get_time=None, extra_cmd=None):
        gr.basic_block.__init__(self, "Msg", None, None)
        self.finished = False
        if len(msgs) != len(periods):
            raise Exception("Message array length must match periods")
        self.msgs = msgs
        self.periods = periods
        self.msg_num = 0
        self.msg_port = pmt.intern("command")
        self.message_port_register_out(self.msg_port)
        self.thread = None
        if add_time:
            self.get_time = get_time
        self.extra_cmd = extra_cmd


    def start(self):
        self.finished = False
        self.thread = Thread(target = self.run)
        self.thread.start()
        return True

    def run(self):
        while not self.finished:
            time.sleep(self.periods[self.msg_num])
            if self.finished:
                return
            msg = self.msgs[self.msg_num]
            timespec = self.get_time() + uhd.time_spec(0.1)
            pmt_time = pmt.cons(
                    pmt.from_long(timespec.get_full_secs()),
                    pmt.from_double(timespec.get_frac_secs())
                    )

            msg = pmt.dict_add(msg, pmt.intern("time"), pmt_time)
            self.message_port_pub(self.msg_port, msg)
            if self.extra_cmd:
                self.extra_cmd(timespec)
            # Needs to be updated after extra_cmd called
            self.msg_num = (self.msg_num + 1) % len(self.msgs)

    def stop(self):
        self.finished = True
        self.thread.join()
        return True

class tworx_usrp_source(gr.hier_block2):

    def __init__(self, samp_rate=1000000, center_freq=2400000000, gain=40, sources=4, addresses="addr0=192.168.10.2, addr1=192.168.20.3", antenna="RX2", num_samps=100000):


        if sources == 2:
            clk_time_src = 'internal'
            subdevs = 'A:AB B:AB'
            antenna_list = ["A", "B"]
        else:
            clk_time_src = 'external'
            subdevs = 'A:0 B:0'
            antenna_list = ["RX2", "TX/RX"]
        self.usrp_sources = sources
        if antenna == "Toggle":
            self.toggle = True
            self.antenna = antenna_list[0]
            self.output_sources = sources * 2
        else:
            self.toggle = False
            self.antenna = antenna
            self.output_sources = sources

        gr.hier_block2.__init__(
            self, "TwoRx USRP",
            gr.io_signature(0, 0, 0),
            gr.io_signaturev(self.output_sources, self.output_sources, gen_sig_io(self.output_sources)),
        )

        ##################################################
        # Parameters
        ##################################################
        self.samp_rate = samp_rate
        self.center_freq = center_freq
        self.gain = gain
        self.addresses = addresses
        self.num_samps = num_samps

        self.msg_port = "command"

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0 = uhd.usrp_source(
                ",".join((self.addresses, "")),
                uhd.stream_args(
                        cpu_format="fc32",
                        channels=range(self.usrp_sources),
                ),
                issue_stream_cmd_on_start=False,
        )
        if self.toggle:
            msgs = list()
            for ant in antenna_list:
                msg = pmt.make_dict()
                msg = pmt.dict_add(msg, pmt.intern("antenna"), pmt.intern(ant))
                msgs.append(msg)
            periods = [0.5, 0.5]
            self.gen_msgs = msg_strobe(msgs=msgs, periods=periods, add_time=True,
                                       get_time=self.uhd_usrp_source_0.get_time_now,
                                       extra_cmd=self.stream_samps)
            self.selectors = list()
            for _ in range(self.usrp_sources):
                self.selectors.append(blocks.selector(gr.sizeof_gr_complex, 0, 0))


        self.uhd_usrp_source_0.set_clock_source(clk_time_src, 0)
        self.uhd_usrp_source_0.set_time_source(clk_time_src, 0)
        if self.usrp_sources == 4:
            self.uhd_usrp_source_0.set_clock_source(clk_time_src, 1)
            self.uhd_usrp_source_0.set_time_source(clk_time_src, 1)
        time.sleep(1)  # Let clocks settle
        self.uhd_usrp_source_0.set_time_unknown_pps(uhd.time_spec())
        time.sleep(1)  # Let clocks settle
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_subdev_spec(subdevs, 0)
        if self.usrp_sources == 4:
            self.uhd_usrp_source_0.set_subdev_spec(subdevs, 1)
        self.uhd_usrp_source_0.set_antenna(self.antenna, 0)
        self.uhd_usrp_source_0.set_antenna(self.antenna, 1)
        if self.usrp_sources == 4:
            self.uhd_usrp_source_0.set_antenna(self.antenna, 2)
            self.uhd_usrp_source_0.set_antenna(self.antenna, 3)

        # Set channel specific settings
        self.uhd_usrp_source_0.set_gain(gain, 0)
        self.uhd_usrp_source_0.set_auto_dc_offset(True, 0)
        self.uhd_usrp_source_0.set_gain(gain, 1)
        self.uhd_usrp_source_0.set_auto_dc_offset(True, 1)
        if self.usrp_sources == 4:
            self.uhd_usrp_source_0.set_gain(gain, 2)
            self.uhd_usrp_source_0.set_auto_dc_offset(True, 2)
            self.uhd_usrp_source_0.set_gain(gain, 3)
            self.uhd_usrp_source_0.set_auto_dc_offset(True, 3)

        # Use timed commands to set frequencies
        self.set_center_freq(center_freq)

        ##################################################
        # Connections
        ##################################################
        if self.toggle:
            for source in range(self.usrp_sources):
                self.connect((self.uhd_usrp_source_0, source), (self.selectors[source], 0))
                self.selectors[source].set_enabled(True)
                self.connect((self.selectors[source], 0), (self, source))
                self.connect((self.selectors[source], 1), (self, self.usrp_sources + source))
            self.msg_connect((self.gen_msgs, 'command'), (self.uhd_usrp_source_0, 'command'))
        else:
            for source in range(self.usrp_sources):
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

    def set_center_freq(self, center_freq):
        # Tune all channels to the desired frequency
        tune_resp = self.uhd_usrp_source_0.set_center_freq(center_freq, 0)
        tune_req = uhd.tune_request(rf_freq=center_freq, rf_freq_policy=uhd.tune_request.POLICY_MANUAL,
               dsp_freq=tune_resp.actual_dsp_freq, dsp_freq_policy=uhd.tune_request.POLICY_MANUAL)

        self.uhd_usrp_source_0.set_center_freq(tune_req, 1)
        if self.usrp_sources==4:
            self.uhd_usrp_source_0.set_center_freq(tune_req, 2)
            self.uhd_usrp_source_0.set_center_freq(tune_req, 3)

        # Synchronize the tuned channels
        now = self.uhd_usrp_source_0.get_time_now()
        self.uhd_usrp_source_0.set_command_time(now + uhd.time_spec(0.1))

        self.uhd_usrp_source_0.set_center_freq(tune_req, 0)
        self.uhd_usrp_source_0.set_center_freq(tune_req, 1)
        if self.usrp_sources==4:
            self.uhd_usrp_source_0.set_center_freq(tune_req, 2)
            self.uhd_usrp_source_0.set_center_freq(tune_req, 3)

        self.uhd_usrp_source_0.clear_command_time()

    def get_center_freq(self):
        return self.center_freq

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        print("DO NOT TUNE GAINS DURING RUNTIME")

    def stream_samps(self, timespec):
        for sel in self.selectors:
            print(self.gen_msgs.msg_num)
            sel.set_output_index(self.gen_msgs.msg_num)
        cmd = uhd.stream_cmd_t(uhd.stream_cmd_t.STREAM_MODE_NUM_SAMPS_AND_DONE)
        cmd.num_samps = self.num_samps
        cmd.time_spec = timespec
        self.uhd_usrp_source_0.issue_stream_cmd(cmd)

