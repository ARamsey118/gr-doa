# Copyright 2011 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
install(FILES
    doa_phase_correct_hier.block.yml
    doa_twinrx_usrp_source.block.yml
    doa_tworx_usrp_source.block.yml
    doa_autocorrelate.block.yml
    doa_MUSIC_lin_array.block.yml
    doa_average_and_save.block.yml
    doa_rootMUSIC_linear_array.block.yml
    doa_qt_compass.block.yml
    doa_antenna_correction.block.yml
    doa_save_antenna_calib.block.yml
    doa_twinrx_phase_offset_est.block.yml
    doa_findmax_and_save.block.yml
    doa_find_local_max.block.yml
    doa_calibrate_lin_array.block.yml
    DESTINATION share/gnuradio/grc/blocks
)
