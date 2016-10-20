project('Elcano C7', 'cpp',
    license        : 'Apache v2.0',
    default_options: ['cpp_std=c++14'],
    version        : '0.0.1')

subdir('serial-1.2.1')

executable('C7',
    sources     : ['main.cc', 'serial-ring.cc'],
    dependencies: serial)
