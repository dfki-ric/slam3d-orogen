module Slam3d

	## Read in parameter set
	def Slam3d.get_params()
		params = ['default']
		if ARGV.length >= 1
			params.push(ARGV[0])
		else
			puts "Too few arguments! USAGE: #{__FILE__} <scenario_profile>"
			exit
		end
		return params
	end

	## Setup the mapper
	def Slam3d.setup_mapper(scan_port, mapper, params)

		Orocos.conf.apply(mapper, params, true)
		Bundles.transformer.setup(mapper)
		mapper.configure
		mapper.start

		## Setup the data connections
		scan_port.connect_to mapper.scan

	end

	## Setup the pointcloud filter
	def Slam3d.setup_filter_mapper(scan_port, filter, mapper, params)

		Orocos.conf.apply(filter, params, true)
		Bundles.transformer.setup(filter)
		filter.configure
		filter.start

		scan_port.connect_to filter.input
		setup_mapper(filter.output, mapper, params)

	end

	## Setup converter to convert from DepthMap to Pointcloud
	def Slam3d.setup_converter_filter_mapper(scan_port, converter, filter, mapper, params)
		converter.configure
		converter.start
	
		scan_port.connect_to converter.depth_map
		setup_filter_mapper(converter.cloud, filter, mapper, params)
	end

end

