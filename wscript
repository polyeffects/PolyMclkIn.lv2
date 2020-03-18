#!/usr/bin/env python
from waflib.extras import autowaf as autowaf
import re

# Variables for 'waf dist'
APPNAME = 'poly_mclk_in.lv2'
VERSION = '1.0.0'

# Mandatory variables
top = '.'
out = 'build'

def options(opt):
    opt.load('compiler_c')
    opt.load('lv2')
    autowaf.set_options(opt)

def configure(conf):
    conf.load('compiler_c', cache=True)
    conf.load('lv2', cache=True)
    conf.load('autowaf', cache=True)

    conf.check_pkg('lv2', uselib_store='LV2')

def build(bld):
    bundle = 'poly_mclk_in.lv2'

    # Build manifest.ttl by substitution (for portable lib extension)
    bld(features     = 'subst',
        source       = 'manifest.ttl.in',
        target       = 'lv2/%s/%s' % (bundle, 'manifest.ttl'),
        install_path = '${LV2DIR}/%s' % bundle,
        LIB_EXT      = bld.env.LV2_LIB_EXT)

    # Copy other data files to build bundle (build/eg-midigate.lv2)
    for i in ['mclk_in.ttl']:
        bld(features     = 'subst',
            is_copy      = True,
            source       = i,
            target       = 'lv2/%s/%s' % (bundle, i),
            install_path = '${LV2DIR}/%s' % bundle)

    # Build plugin library
    obj = bld(features     = 'c cshlib lv2lib',
              source       = 'mclk_in.c',
              name         = 'mclk',
              target       = 'lv2/%s/mclk_in' % bundle,
              install_path = '${LV2DIR}/%s' % bundle,
              uselib       = 'LV2')
