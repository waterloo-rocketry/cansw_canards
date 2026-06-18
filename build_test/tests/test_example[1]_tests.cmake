add_test( ExampleTest.DescriptiveTestNameYup /workspaces/cansw_processor_canards/build_test/tests/test_example [==[--gtest_filter=ExampleTest.DescriptiveTestNameYup]==] --gtest_also_run_disabled_tests)
set_tests_properties( ExampleTest.DescriptiveTestNameYup PROPERTIES WORKING_DIRECTORY /workspaces/cansw_processor_canards/build_test/tests SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set( test_example_TESTS ExampleTest.DescriptiveTestNameYup)
