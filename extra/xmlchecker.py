import xmlschema

xmlschema.validate(r'..\data\surveys\demo\tls_arbaro_demo.xml', 'survey.xsd')
xmlschema.validate(r'..\data\scenes\demo\arbaro_demo.xml', 'scene.xsd')
xmlschema.validate(r'..\data\scanners_als.xml', 'scanner.xsd')
xmlschema.validate(r'..\data\platforms.xml', 'platform.xsd')
